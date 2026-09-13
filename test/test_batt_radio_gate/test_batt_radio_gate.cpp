#include <gtest/gtest.h>

#include <helpers/BattRadioGate.h>
#include <helpers/DynamicConfigSerializer.h>

// 600 s hold, 360 min minimum ON window
static void configure(BattRadioGate& g) {
  char reply[160];
  ASSERT_TRUE(g.handleCommand("set batt.gate on_mv=4000,off_mv=3400,min_on=360,hold=600", reply, sizeof(reply)));
  ASSERT_TRUE(g.handleCommand("set batt.gate on", reply, sizeof(reply)));
}

TEST(BattRadioGate, disabledKeepsRadioOn) {
  BattRadioGate g;
  g.begin(3000, false, 0, 0);
  EXPECT_TRUE(g.isOn());
  EXPECT_FALSE(g.update(2000, false, 1000000, 0));
  EXPECT_TRUE(g.isOn());
}

TEST(BattRadioGate, disabledIsCompletelyInert) {
  DynamicConfigSerializer store;
  BattRadioGate g;
  g.load(store);
  ASSERT_FALSE(g.isEnabled());

  g.begin(4100, false, 0, 0);
  EXPECT_TRUE(g.isOn());
  EXPECT_FALSE(g.isDirty());

  EXPECT_FALSE(g.update(2000, false, 10000000, 0));
  EXPECT_FALSE(g.isDirty());

  EXPECT_FALSE(g.save());                       // nothing to flush
  char v[8];
  EXPECT_FALSE(store.getByKey("bg_st", v, sizeof(v)));   // no keys created
  EXPECT_FALSE(store.getByKey("bg_cyc", v, sizeof(v)));
}

TEST(BattRadioGate, needsOnThresholdHeldBeforeStarting) {
  BattRadioGate g;
  configure(g);
  g.begin(3000, false, 0, 100);
  EXPECT_FALSE(g.isOn());

  EXPECT_FALSE(g.update(4100, false, 0, 100));        // just crossed
  EXPECT_FALSE(g.update(4100, false, 599000, 100));   // hold not elapsed
  EXPECT_TRUE(g.update(4100, false, 600000, 200));    // hold elapsed -> ON
  EXPECT_TRUE(g.isOn());
  EXPECT_EQ(1u, g.cycles);
  EXPECT_EQ(200u, g.last_cycle);
}

TEST(BattRadioGate, sagAndRecoveryDoNotBounce) {
  BattRadioGate g;
  configure(g);
  g.begin(4100, false, 0, 0);   // comes up charged
  ASSERT_TRUE(g.isOn());

  EXPECT_FALSE(g.update(3200, false, 1000, 0));    // TX-like dip
  EXPECT_FALSE(g.update(4100, false, 2000, 0));    // recovered
  EXPECT_TRUE(g.isOn());
  EXPECT_EQ(0u, g.cycles);                          // no spurious cycle
}

TEST(BattRadioGate, deadBandHoldsCurrentState) {
  BattRadioGate g;
  configure(g);

  g.begin(3000, false, 0, 0);
  EXPECT_FALSE(g.isOn());
  EXPECT_FALSE(g.update(3700, false, 10000000, 0));   // inside dead band
  EXPECT_FALSE(g.isOn());

  BattRadioGate h;
  configure(h);
  h.begin(4100, false, 0, 0);
  ASSERT_TRUE(h.isOn());
  EXPECT_FALSE(h.update(3700, false, 10000000, 0));
  EXPECT_TRUE(h.isOn());
}

TEST(BattRadioGate, minOnWindowDelaysParking) {
  BattRadioGate g;
  configure(g);
  g.begin(4100, false, 0, 0);
  ASSERT_TRUE(g.isOn());

  EXPECT_FALSE(g.update(3300, false, 600000, 0));            // candidate OFF
  EXPECT_FALSE(g.update(3300, false, 1200000, 0));           // hold met, min_on not
  EXPECT_TRUE(g.isOn());

  EXPECT_TRUE(g.update(3300, false, 21600000, 0));           // 360 min elapsed
  EXPECT_FALSE(g.isOn());
}

TEST(BattRadioGate, implausibleReadingsHoldState) {
  BattRadioGate g;
  configure(g);
  g.begin(4100, false, 0, 0);
  ASSERT_TRUE(g.isOn());

  EXPECT_FALSE(g.update(0, false, 21600000, 0));       // dead ADC
  EXPECT_FALSE(g.update(5000, false, 21600000, 0));    // above any Li-ion cell
  EXPECT_TRUE(g.isOn());
}

TEST(BattRadioGate, externalPowerOverridesEverything) {
  BattRadioGate g;
  configure(g);
  g.begin(3000, false, 0, 0);
  ASSERT_FALSE(g.isOn());

  EXPECT_TRUE(g.update(3000, true, 0, 0));   // USB present -> radio on
  EXPECT_TRUE(g.isOn());
}

TEST(BattRadioGate, cycleCountSurvivesReboot) {
  DynamicConfigSerializer store;
  BattRadioGate g;
  g.load(store);
  configure(g);
  g.begin(3000, false, 0, 100);
  ASSERT_FALSE(g.isOn());
  g.save();

  // recharge while parked, then a cycle completes
  g.update(4100, false, 0, 100);
  g.update(4100, false, 600000, 777);
  ASSERT_TRUE(g.isOn());
  ASSERT_EQ(1u, g.cycles);
  g.save();

  // reboot with a charged cell: must not double count
  BattRadioGate g2;
  g2.load(store);
  g2.begin(4100, false, 0, 999);
  EXPECT_TRUE(g2.isOn());
  EXPECT_EQ(1u, g2.cycles);
  EXPECT_EQ(777u, g2.last_cycle);
  g2.save();

  // reboot parked (cell dropped), then a recharge completes -> cycle 2
  BattRadioGate g3;
  g3.load(store);
  g3.begin(3000, false, 0, 0);
  EXPECT_FALSE(g3.isOn());
  g3.save();

  BattRadioGate g4;
  g4.load(store);
  g4.begin(4100, false, 0, 0);
  EXPECT_TRUE(g4.isOn());
  EXPECT_EQ(2u, g4.cycles);
}

TEST(BattRadioGate, commandParsingAndValidation) {
  BattRadioGate g;
  char reply[160];

  EXPECT_FALSE(g.handleCommand("get radio", reply, sizeof(reply)));

  ASSERT_TRUE(g.handleCommand("set batt.gate on_mv=4100,off_mv=3500,min_on=120,hold=300", reply, sizeof(reply)));
  EXPECT_EQ(4100, g.on_mv);
  EXPECT_EQ(3500, g.off_mv);
  EXPECT_EQ(120, g.min_on_mins);
  EXPECT_EQ(300, g.hold_secs);

  ASSERT_TRUE(g.handleCommand("get batt.gate", reply, sizeof(reply)));
  EXPECT_NE(nullptr, strstr(reply, "on_mv=4100"));

  // inverted thresholds are rejected and leave the config untouched
  ASSERT_TRUE(g.handleCommand("set batt.gate on_mv=3300,off_mv=3400", reply, sizeof(reply)));
  EXPECT_EQ(0, strncmp(reply, "Err", 3));
  EXPECT_EQ(4100, g.on_mv);
  EXPECT_EQ(3500, g.off_mv);

  // unknown field is rejected
  ASSERT_TRUE(g.handleCommand("set batt.gate bogus=1", reply, sizeof(reply)));
  EXPECT_EQ(0, strncmp(reply, "Err", 3));

  ASSERT_TRUE(g.handleCommand("set batt.gate reset", reply, sizeof(reply)));
  EXPECT_EQ(0u, g.cycles);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
