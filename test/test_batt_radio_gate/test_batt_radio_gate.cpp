#include <gtest/gtest.h>

#include <helpers/BattRadioGate.h>
#include <helpers/DynamicConfigSerializer.h>

#define CHG_UNKNOWN (-1)

static bool cmd(BattRadioGate& g, const char* c) {
  char reply[160];
  return g.handleCommand(c, reply, sizeof(reply));
}

// voltage gate on, charge gate off, both holds 600 s for deterministic times
static void configure(BattRadioGate& g) {
  ASSERT_TRUE(cmd(g, "set batt.gate on_mv=4000,off_mv=3400,min_on=360,hold=600"));
  ASSERT_TRUE(cmd(g, "set batt.gate on"));
  ASSERT_TRUE(cmd(g, "set batt.chg off"));
}

// charge gate alone, voltage gate off, min_on 0 so only the charge hold matters
static void configureChgOnly(BattRadioGate& g) {
  ASSERT_TRUE(cmd(g, "set batt.gate on_mv=4000,off_mv=3400,min_on=0,hold=600"));
  ASSERT_TRUE(cmd(g, "set batt.gate off"));
  ASSERT_TRUE(cmd(g, "set batt.chg on"));
  ASSERT_TRUE(cmd(g, "set batt.chg hold=600"));
  ASSERT_FALSE(g.isEnabled());
  ASSERT_TRUE(g.isChgEnabled());
}

// both gates on
static void configureBoth(BattRadioGate& g) {
  ASSERT_TRUE(cmd(g, "set batt.gate on_mv=4000,off_mv=3400,min_on=360,hold=600"));
  ASSERT_TRUE(cmd(g, "set batt.gate on"));
  ASSERT_TRUE(cmd(g, "set batt.chg on"));
  ASSERT_TRUE(cmd(g, "set batt.chg hold=600"));
}

// feed `count` readings spaced `step_ms` apart, advancing `now`
static void feed(BattRadioGate& g, uint16_t mv, int32_t chg, uint32_t& now,
                 uint32_t count, uint32_t step_ms) {
  for (uint32_t i = 0; i < count; i++) {
    g.update(mv, false, chg, now, now);
    now += step_ms;
  }
}

TEST(BattRadioGate, bothGatesOffIsInert) {
  DynamicConfigSerializer store;
  BattRadioGate g;
  g.load(store);
  ASSERT_FALSE(g.isActive());

  g.begin(3000, false, 0, 0, 0);
  EXPECT_TRUE(g.isOn());                       // nothing switched on -> radio runs
  EXPECT_FALSE(g.isDirty());

  EXPECT_FALSE(g.update(2000, false, 0, 10000000, 0));
  EXPECT_TRUE(g.isOn());
  EXPECT_FALSE(g.isDirty());

  EXPECT_FALSE(g.save());
  char v[8];
  EXPECT_FALSE(store.getByKey("bg_st", v, sizeof(v)));   // no keys created
  EXPECT_FALSE(store.getByKey("bg_cyc", v, sizeof(v)));
}

TEST(BattRadioGate, voltageGateNeedsThresholdHeldBeforeStarting) {
  BattRadioGate g;
  configure(g);
  g.begin(3000, false, CHG_UNKNOWN, 0, 100);
  EXPECT_FALSE(g.isOn());

  EXPECT_FALSE(g.update(4100, false, CHG_UNKNOWN, 0, 100));        // just crossed
  EXPECT_FALSE(g.update(4100, false, CHG_UNKNOWN, 599000, 100));   // hold not elapsed
  EXPECT_TRUE(g.update(4100, false, CHG_UNKNOWN, 600000, 200));    // hold elapsed -> ON
  EXPECT_TRUE(g.isOn());
  EXPECT_EQ(1u, g.cycles);
}

TEST(BattRadioGate, sagAndRecoveryDoNotBounce) {
  BattRadioGate g;
  configure(g);
  g.begin(4100, false, CHG_UNKNOWN, 0, 0);   // comes up charged
  ASSERT_TRUE(g.isOn());

  EXPECT_FALSE(g.update(3200, false, CHG_UNKNOWN, 1000, 0));    // TX-like dip
  EXPECT_FALSE(g.update(4100, false, CHG_UNKNOWN, 2000, 0));    // recovered
  EXPECT_TRUE(g.isOn());
  EXPECT_EQ(0u, g.cycles);                          // no spurious cycle
}

TEST(BattRadioGate, deadBandHoldsCurrentState) {
  BattRadioGate g;
  configure(g);

  g.begin(3000, false, CHG_UNKNOWN, 0, 0);
  EXPECT_FALSE(g.isOn());
  EXPECT_FALSE(g.update(3700, false, CHG_UNKNOWN, 10000000, 0));   // inside dead band
  EXPECT_FALSE(g.isOn());

  BattRadioGate h;
  configure(h);
  h.begin(4100, false, CHG_UNKNOWN, 0, 0);
  ASSERT_TRUE(h.isOn());
  EXPECT_FALSE(h.update(3700, false, CHG_UNKNOWN, 10000000, 0));
  EXPECT_TRUE(h.isOn());
}

TEST(BattRadioGate, minOnWindowDelaysParking) {
  BattRadioGate g;
  configure(g);
  g.begin(4100, false, CHG_UNKNOWN, 0, 0);
  ASSERT_TRUE(g.isOn());

  EXPECT_FALSE(g.update(3300, false, CHG_UNKNOWN, 600000, 0));            // candidate OFF
  EXPECT_FALSE(g.update(3300, false, CHG_UNKNOWN, 1200000, 0));           // hold met, min_on not
  EXPECT_TRUE(g.isOn());

  EXPECT_TRUE(g.update(3300, false, CHG_UNKNOWN, 21600000, 0));           // 360 min elapsed
  EXPECT_FALSE(g.isOn());
}

TEST(BattRadioGate, implausibleReadingsHoldState) {
  BattRadioGate g;
  configure(g);
  g.begin(4100, false, CHG_UNKNOWN, 0, 0);
  ASSERT_TRUE(g.isOn());

  EXPECT_FALSE(g.update(0, false, CHG_UNKNOWN, 21600000, 0));       // dead ADC
  EXPECT_FALSE(g.update(5000, false, CHG_UNKNOWN, 21600000, 0));    // above any Li-ion cell
  EXPECT_TRUE(g.isOn());
}

TEST(BattRadioGate, externalPowerOverridesEverything) {
  BattRadioGate g;
  configure(g);
  g.begin(3000, false, CHG_UNKNOWN, 0, 0);
  ASSERT_FALSE(g.isOn());

  EXPECT_TRUE(g.update(3000, true, CHG_UNKNOWN, 0, 0));   // USB present -> radio on
  EXPECT_TRUE(g.isOn());
}

// ---- charge gate, independent of the voltage gate ----

TEST(BattRadioGate, chargeGateAloneDoesNotNeedTheVoltageGate) {
  BattRadioGate g;
  configureChgOnly(g);
  uint32_t now = 0;

  g.begin(3700, false, 0, now, 0);          // mid-band cell, no harvest
  ASSERT_FALSE(g.isOn());

  feed(g, 3700, 250000, now, 1, 60000);     // harvest starts at t=0
  EXPECT_FALSE(g.isOn());
  now = 599000; feed(g, 3700, 250000, now, 1, 60000);
  EXPECT_FALSE(g.isOn());                   // hold not elapsed
  now = 600000; feed(g, 3700, 250000, now, 1, 60000);
  EXPECT_TRUE(g.isOn());                    // sustained -> ON

  // harvest stops (min_on is 0 here): the charge gate parks it again, the cell
  // is still at 3.7 V and the voltage gate is off, so nothing else objects
  now = 600000; feed(g, 3700, 0, now, 4, 60000);   // average needs the full window
  EXPECT_FALSE(g.update(3700, false, 0, now, 0));
  now += 600000;
  EXPECT_TRUE(g.update(3700, false, 0, now, 0));
  EXPECT_FALSE(g.isOn());
}

TEST(BattRadioGate, chargeGateBlocksStartWithoutHarvest) {
  BattRadioGate g;
  configureBoth(g);
  uint32_t now = 0;

  g.begin(4100, false, 0, now, 0);          // full cell, but nothing coming in
  EXPECT_FALSE(g.isOn());

  feed(g, 4100, 0, now, 10, 60000);
  EXPECT_FALSE(g.isOn());
  EXPECT_FALSE(g.update(4100, false, CHG_UNKNOWN, now, 0));   // unknown is conservative
  EXPECT_FALSE(g.isOn());
}

TEST(BattRadioGate, voltageGateStillParksWhileHarvesting) {
  BattRadioGate g;
  configureBoth(g);
  g.begin(4100, false, 250000, 0, 0);     // charged and harvesting -> active
  ASSERT_TRUE(g.isOn());

  // cell falls: the voltage gate must park it even though harvest continues
  EXPECT_FALSE(g.update(3300, false, 250000, 600000, 0));
  EXPECT_TRUE(g.update(3300, false, 250000, 21600000, 0));
  EXPECT_FALSE(g.isOn());
}

TEST(BattRadioGate, voltageGateWithLowCellOutranksHarvest) {
  BattRadioGate g;
  configureBoth(g);
  g.begin(3000, false, 250000, 0, 0);
  EXPECT_FALSE(g.isOn());
  EXPECT_FALSE(g.update(3000, false, 250000, 21600000, 0));
  EXPECT_FALSE(g.isOn());
}

// ---- weather and noise ----

TEST(BattRadioGate, passingCloudDoesNotParkTheRadio) {
  BattRadioGate g;
  configureChgOnly(g);
  uint32_t now = 0;
  g.begin(3700, false, 250000, now, 0);      // sun at boot -> active
  ASSERT_TRUE(g.isOn());

  // 8 minutes of cloud, then the sun returns
  feed(g, 3700, 0, now, 8, 60000);
  EXPECT_TRUE(g.isOn());
  feed(g, 3700, 250000, now, 4, 60000);
  EXPECT_TRUE(g.isOn());
}

TEST(BattRadioGate, alternatingSunAndCloudNeverParks) {
  BattRadioGate g;
  configureChgOnly(g);
  uint32_t now = 0;
  g.begin(3700, false, 250000, now, 0);
  ASSERT_TRUE(g.isOn());

  // 12 min of cloud per cycle is LONGER than the 10 min hold, but shorter than
  // hold + the 4-sample averaging window - so this only holds because the
  // average, not a single sample, drives the timer. Without the averaging the
  // radio would park on the very first cycle.
  for (int cycle = 0; cycle < 5; cycle++) {
    feed(g, 3700, 0, now, 12, 60000);
    EXPECT_TRUE(g.isOn()) << "parked during cloud at cycle " << cycle;
    feed(g, 3700, 250000, now, 3, 60000);
    EXPECT_TRUE(g.isOn()) << "parked during sun at cycle " << cycle;
  }
}

TEST(BattRadioGate, pulsedHarvestCurrentStillOpensTheGate) {
  BattRadioGate g;
  configureChgOnly(g);
  uint32_t now = 0;
  g.begin(3700, false, 0, now, 0);           // starts parked
  ASSERT_FALSE(g.isOn());

  // a harvester PMIC delivers a pulsed current: alternating 250 uA / 0 every
  // 30 s must not starve the gate, because the average stays positive
  for (int i = 0; i < 40; i++) {
    g.update(3700, false, 250000, now, now); now += 30000;
    g.update(3700, false, 0,      now, now); now += 30000;
  }
  EXPECT_TRUE(g.isOn());
}

TEST(BattRadioGate, sustainedCloudParksTheRadio) {
  BattRadioGate g;
  configureChgOnly(g);
  uint32_t now = 0;
  g.begin(3700, false, 250000, now, 0);
  ASSERT_TRUE(g.isOn());

  // a genuinely overcast half hour: average drops within the window, hold runs
  feed(g, 3700, 0, now, 30, 60000);
  EXPECT_FALSE(g.isOn());
}

TEST(BattRadioGate, failedI2CReadIsNotCountedAsNoSun) {
  BattRadioGate g;
  configureChgOnly(g);
  uint32_t now = 0;
  g.begin(3700, false, 250000, now, 0);
  ASSERT_TRUE(g.isOn());

  // ten consecutive I2C failures must read as "no news", not "no sun"
  feed(g, 3700, CHG_UNKNOWN, now, 10, 60000);
  EXPECT_TRUE(g.isOn());
}

TEST(BattRadioGate, chargeGateHoldIsFloored) {
  BattRadioGate g;
  configureChgOnly(g);
  ASSERT_TRUE(cmd(g, "set batt.chg hold=0"));
  EXPECT_EQ(0, g.chg_hold_secs);
  uint32_t now = 0;
  g.begin(3700, false, 0, now, 0);           // no harvest -> parked
  ASSERT_FALSE(g.isOn());

  g.update(3700, false, 250000, 0, 0);                       // candidate ON from t=0
  g.update(3700, false, 250000, BattRadioGate::CHG_MIN_HOLD_MS - 1000, 0);
  EXPECT_FALSE(g.isOn());                                    // floor, not hold=0
  g.update(3700, false, 250000, BattRadioGate::CHG_MIN_HOLD_MS, 0);
  EXPECT_TRUE(g.isOn());
}

// ---- switching the two gates independently ----

TEST(BattRadioGate, switchingOneGateOffLeavesTheOtherRunning) {
  BattRadioGate g;
  configureBoth(g);

  ASSERT_TRUE(cmd(g, "set batt.gate off"));
  EXPECT_FALSE(g.isEnabled());
  EXPECT_TRUE(g.isChgEnabled());
  EXPECT_TRUE(g.isActive());

  ASSERT_TRUE(cmd(g, "set batt.chg off"));
  EXPECT_FALSE(g.isActive());     // both off -> inert again
}

TEST(BattRadioGate, bothSwitchesAndHoldsPersistAcrossReboot) {
  DynamicConfigSerializer store;
  BattRadioGate g;
  g.load(store);
  configureChgOnly(g);            // voltage off, charge on, chg hold 600
  g.save();

  BattRadioGate g2;
  g2.load(store);
  EXPECT_FALSE(g2.isEnabled());
  EXPECT_EQ(1, g2.chg_enabled);
  EXPECT_EQ(600, g2.chg_hold_secs);
  EXPECT_TRUE(g2.isActive());

  ASSERT_TRUE(cmd(g2, "set batt.chg off"));
  g2.save();

  BattRadioGate g3;
  g3.load(store);
  EXPECT_FALSE(g3.isEnabled());
  EXPECT_FALSE(g3.chg_enabled);
  EXPECT_FALSE(g3.isActive());
}

TEST(BattRadioGate, disablingOneGateReleasesItsVeto) {
  // parked by the voltage gate: harvest alone must NOT bring the radio back
  // while the voltage gate is still on...
  BattRadioGate a;
  configureBoth(a);
  a.begin(3000, false, 250000, 0, 0);
  ASSERT_FALSE(a.isOn());
  a.update(3000, false, 250000, 60000, 60000);
  EXPECT_FALSE(a.isOn());

  // ...but switching that gate off releases it on the next sample
  ASSERT_TRUE(cmd(a, "set batt.gate off"));
  EXPECT_TRUE(a.update(3000, false, 250000, 120000, 120000));
  EXPECT_TRUE(a.isOn());

  // same the other way round: full cell parked by the charge gate comes back
  // as soon as the charge gate is switched off
  BattRadioGate b;
  configureBoth(b);
  b.begin(4100, false, 0, 0, 0);
  ASSERT_FALSE(b.isOn());
  b.update(4100, false, 0, 60000, 60000);
  EXPECT_FALSE(b.isOn());

  ASSERT_TRUE(cmd(b, "set batt.chg off"));
  EXPECT_TRUE(b.update(4100, false, 0, 120000, 120000));
  EXPECT_TRUE(b.isOn());
}

TEST(BattRadioGate, bothGatesActiveNeedBothToAllow) {
  BattRadioGate g;
  configureBoth(g);
  EXPECT_TRUE(g.isEnabled());
  EXPECT_TRUE(g.isChgEnabled());

  // charged but not harvesting -> stays parked
  g.begin(4100, false, 0, 0, 0);
  EXPECT_FALSE(g.isOn());
  // harvesting but cell low -> stays parked
  BattRadioGate h;
  configureBoth(h);
  h.begin(3000, false, 250000, 0, 0);
  EXPECT_FALSE(h.isOn());
  // both satisfied -> active
  BattRadioGate k;
  configureBoth(k);
  k.begin(4100, false, 250000, 0, 0);
  EXPECT_TRUE(k.isOn());
}

TEST(BattRadioGate, chargeGateMinCurrentBlocksWeakHarvest) {
  BattRadioGate g;
  configureChgOnly(g);
  ASSERT_TRUE(cmd(g, "set batt.chg min=0.2"));
  ASSERT_EQ(200, g.chgMinUa());

  uint32_t now = 0;
  g.begin(3700, false, 100000, now, 0);      // 0.1 mA: positive, but below the bar
  EXPECT_FALSE(g.isOn());

  feed(g, 3700, 100000, now, 30, 60000);     // half an hour of it
  EXPECT_FALSE(g.isOn());

  feed(g, 3700, 250000, now, 30, 60000);     // 0.25 mA -> clearly above -> opens
  EXPECT_TRUE(g.isOn());
}

TEST(BattRadioGate, chargeGateMinCurrentSnapsAndPersists) {
  DynamicConfigSerializer store;
  BattRadioGate g;
  char reply[160];
  g.load(store);

  ASSERT_TRUE(cmd(g, "set batt.chg min=0.06"));          // snaps to 0.05 mA
  EXPECT_EQ(50, g.chgMinUa());
  ASSERT_TRUE(g.handleCommand("get batt.chg", reply, sizeof(reply)));
  EXPECT_NE(nullptr, strstr(reply, "min=0.05mA"));

  ASSERT_TRUE(cmd(g, "set batt.chg min=1"));             // whole mA stays whole
  EXPECT_EQ(1000, g.chgMinUa());
  ASSERT_TRUE(g.handleCommand("get batt.chg", reply, sizeof(reply)));
  EXPECT_NE(nullptr, strstr(reply, "min=1mA"));

  ASSERT_TRUE(cmd(g, "set batt.chg min=8"));             // snaps to the top entry
  EXPECT_EQ(10000, g.chgMinUa());                         // 10 mA, the table max

  ASSERT_TRUE(cmd(g, "set batt.chg min=0.5"));
  g.save();
  BattRadioGate g2;
  g2.load(store);
  EXPECT_EQ(500, g2.chgMinUa());                          // survived the reboot

  ASSERT_TRUE(g2.handleCommand("set batt.chg min=-5", reply, sizeof(reply)));
  EXPECT_EQ(0, strncmp(reply, "Err", 3));
  EXPECT_EQ(500, g2.chgMinUa());

  // above the PMIC's measurable ceiling: must be rejected, not silently capped,
  // because such a threshold could never be met and would hold the gate shut
  ASSERT_TRUE(g2.handleCommand("set batt.chg min=20", reply, sizeof(reply)));
  EXPECT_EQ(0, strncmp(reply, "Err", 3));
  ASSERT_TRUE(g2.handleCommand("set batt.chg min=50", reply, sizeof(reply)));
  EXPECT_EQ(0, strncmp(reply, "Err", 3));
  ASSERT_TRUE(g2.handleCommand("set batt.chg min=2000", reply, sizeof(reply)));
  EXPECT_EQ(0, strncmp(reply, "Err", 3));
  EXPECT_EQ(500, g2.chgMinUa());
}

TEST(BattRadioGate, inertGateReportsTheRadioOn) {
  BattRadioGate g;
  configureChgOnly(g);
  g.begin(3700, false, 0, 0, 0);      // no harvest -> parked
  ASSERT_FALSE(g.isOn());

  // switching the gate off must not leave the caller believing the radio is
  // still parked (the wiring in MyMesh::loop() releases it on this transition)
  ASSERT_TRUE(cmd(g, "set batt.chg off"));
  EXPECT_FALSE(g.isActive());
  EXPECT_TRUE(g.isOn());
}

TEST(BattRadioGate, cycleCountSurvivesReboot) {
  DynamicConfigSerializer store;
  BattRadioGate g;
  g.load(store);
  configure(g);
  g.begin(3000, false, CHG_UNKNOWN, 0, 100);
  ASSERT_FALSE(g.isOn());
  g.save();

  // recharge while parked, then a cycle completes
  g.update(4100, false, CHG_UNKNOWN, 0, 100);
  g.update(4100, false, CHG_UNKNOWN, 600000, 200);
  ASSERT_TRUE(g.isOn());
  ASSERT_EQ(1u, g.cycles);
  g.save();

  // reboot with a charged cell: must not double count
  BattRadioGate g2;
  g2.load(store);
  g2.begin(4100, false, CHG_UNKNOWN, 0, 999);
  EXPECT_TRUE(g2.isOn());
  EXPECT_EQ(1u, g2.cycles);
  g2.save();

  // reboot parked (cell dropped), then a recharge completes -> cycle 2
  BattRadioGate g3;
  g3.load(store);
  g3.begin(3000, false, CHG_UNKNOWN, 0, 0);
  EXPECT_FALSE(g3.isOn());
  g3.save();

  BattRadioGate g4;
  g4.load(store);
  g4.begin(4100, false, CHG_UNKNOWN, 0, 0);
  EXPECT_TRUE(g4.isOn());
  EXPECT_EQ(2u, g4.cycles);
}

TEST(BattRadioGate, commandParsingAndValidation) {
  BattRadioGate g;
  char reply[160];

  EXPECT_FALSE(g.handleCommand("get radio", reply, sizeof(reply)));

  ASSERT_TRUE(cmd(g, "set batt.gate on_mv=4100,off_mv=3500,min_on=120,hold=300"));
  EXPECT_EQ(4100, g.on_mv);
  EXPECT_EQ(3500, g.off_mv);
  EXPECT_EQ(120, g.min_on_mins);
  EXPECT_EQ(300, g.hold_secs);

  ASSERT_TRUE(g.handleCommand("get batt.gate", reply, sizeof(reply)));
  EXPECT_NE(nullptr, strstr(reply, "on_mv=4100"));

  ASSERT_TRUE(g.handleCommand("get batt.chg", reply, sizeof(reply)));
  EXPECT_NE(nullptr, strstr(reply, "off"));
  EXPECT_NE(nullptr, strstr(reply, "hold=900"));
  EXPECT_NE(nullptr, strstr(reply, "min=0mA"));

  // the charge gate is its own command, not a field of set batt.gate
  ASSERT_TRUE(g.handleCommand("set batt.gate chg=on", reply, sizeof(reply)));
  EXPECT_EQ(0, strncmp(reply, "Err", 3));

  // inverted thresholds are rejected and leave the config untouched
  ASSERT_TRUE(g.handleCommand("set batt.gate on_mv=3300,off_mv=3400", reply, sizeof(reply)));
  EXPECT_EQ(0, strncmp(reply, "Err", 3));
  EXPECT_EQ(4100, g.on_mv);
  EXPECT_EQ(3500, g.off_mv);

  ASSERT_TRUE(g.handleCommand("set batt.gate bogus=1", reply, sizeof(reply)));
  EXPECT_EQ(0, strncmp(reply, "Err", 3));

  // chg takes on|off|hold=<secs>
  ASSERT_TRUE(g.handleCommand("set batt.chg maybe", reply, sizeof(reply)));
  EXPECT_EQ(0, strncmp(reply, "Err", 3));
  EXPECT_EQ(0, g.chg_enabled);
  ASSERT_TRUE(cmd(g, "set batt.chg on"));
  EXPECT_EQ(1, g.chg_enabled);
  ASSERT_TRUE(cmd(g, "set batt.chg hold=1200"));
  EXPECT_EQ(1200, g.chg_hold_secs);
  ASSERT_TRUE(g.handleCommand("set batt.chg hold=99999", reply, sizeof(reply)));
  EXPECT_EQ(0, strncmp(reply, "Err", 3));
  EXPECT_EQ(1200, g.chg_hold_secs);
  ASSERT_TRUE(cmd(g, "set batt.chg off"));
  EXPECT_EQ(0, g.chg_enabled);

  ASSERT_TRUE(cmd(g, "set batt.gate reset"));
  EXPECT_EQ(0u, g.cycles);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
