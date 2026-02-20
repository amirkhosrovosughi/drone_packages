#include <gtest/gtest.h>

#include "association/nearest_neighbor_association.hpp"
#include "measurement/point3d_measurement_model.hpp"
#include "common/mock_slam_logger.hpp"

#include <condition_variable>
#include <future>
#include <chrono>
#include <cmath>

using namespace slam;

TEST(NearestNeighborAssociationTest, CreatesThenMatchesLandmark)
{
    NearestNeighborAssociation assoc;
    assoc.setLogger(std::make_shared<MockSlamLogger>());

    std::promise<AssignedMeasurements> prom;
    auto fut = prom.get_future();

    assoc.registerCallback([&prom](AssignedMeasurements am)
                           {
        try {
            prom.set_value(am);
        } catch (...) {
            // ignore if promise already satisfied
        } });

    // Build a single point measurement at (1,2,3)
    Measurement meas;
    meas.payload = Eigen::VectorXd(3);
    meas.payload << 1.0, 2.0, 3.0;
    meas.model = std::make_shared<Point3DMeasurementModel>();

    // First call: should create a new landmark
    assoc.onReceiveMeasurement(Measurements{meas});
    // give async task some time to run; wait up to 5s for callback
    auto status = fut.wait_for(std::chrono::seconds(5));
    ASSERT_EQ(status, std::future_status::ready);
    AssignedMeasurements result = fut.get();
    ASSERT_EQ(result.size(), 1u);
    EXPECT_TRUE(result[0].isNew);
    int firstId = result[0].id;

    // Prepare for second call; expect it to match the existing landmark
    // prepare a fresh promise for the second call
    std::promise<AssignedMeasurements> prom2;
    auto fut2 = prom2.get_future();
    assoc.registerCallback([&prom2](AssignedMeasurements am)
                           {
        try { prom2.set_value(am); } catch(...) {} });
    assoc.onReceiveMeasurement(Measurements{meas});
    auto status2 = fut2.wait_for(std::chrono::seconds(5));
    ASSERT_EQ(status2, std::future_status::ready);
    AssignedMeasurements result2 = fut2.get();
    ASSERT_EQ(result2.size(), 1u);
    EXPECT_FALSE(result2[0].isNew);
    EXPECT_EQ(result2[0].id, firstId);
}

TEST(NearestNeighborAssociationTest, EuclideanDistanceAndHandleUpdate)
{
    // expose protected euclideanDistance via derived test class
    struct TestAssoc : public NearestNeighborAssociation
    {
        using NearestNeighborAssociation::euclideanDistance;
        using NearestNeighborAssociation::handleUpdate;
    } assoc;
    assoc.setLogger(std::make_shared<MockSlamLogger>());

    // simple euclidean distance check
    Landmark a;
    Landmark b;
    a.position = Position(0.0, 0.0, 0.0);
    b.position = Position(3.0, 4.0, 0.0);
    double d = assoc.euclideanDistance(a, b);
    EXPECT_NEAR(d, 5.0, 1e-9);

    // Now test handleUpdate: create initial landmark at (0,0,0)
    std::promise<AssignedMeasurements> prom;
    auto fut = prom.get_future();
    assoc.registerCallback([&prom](AssignedMeasurements am)
                           { try { prom.set_value(am);} catch(...){} });
    Measurement meas;
    meas.payload = Eigen::VectorXd(3);
    meas.payload << 0.0, 0.0, 0.0;
    meas.model = std::make_shared<Point3DMeasurementModel>();
    assoc.onReceiveMeasurement(Measurements{meas});
    ASSERT_EQ(fut.wait_for(std::chrono::seconds(5)), std::future_status::ready);
    AssignedMeasurements res = fut.get();
    ASSERT_EQ(res.size(), 1u);
    int initialId = res[0].id;

    // Create a map where the landmark moved to (10,0,0)
    MapSummary map;
    map.robot = RobotState();
    Landmark lm;
    lm.id = initialId;
    lm.position = Position(10.0, 0.0, 0.0);
    map.landmarks.push_back(lm);

    // Call handleUpdate to apply the new landmark position
    assoc.handleUpdate(map);

    // Now send measurement near (10,0,0) and expect a match (not new)
    std::promise<AssignedMeasurements> prom2;
    auto fut2 = prom2.get_future();
    assoc.registerCallback([&prom2](AssignedMeasurements am)
                           { try { prom2.set_value(am);} catch(...){} });
    Measurement meas2;
    meas2.payload = Eigen::VectorXd(3);
    meas2.payload << 10.0, 0.0, 0.0;
    meas2.model = std::make_shared<Point3DMeasurementModel>();
    assoc.onReceiveMeasurement(Measurements{meas2});
    ASSERT_EQ(fut2.wait_for(std::chrono::seconds(5)), std::future_status::ready);
    AssignedMeasurements res2 = fut2.get();
    ASSERT_EQ(res2.size(), 1u);
    EXPECT_FALSE(res2[0].isNew);
    EXPECT_EQ(res2[0].id, initialId);
}
