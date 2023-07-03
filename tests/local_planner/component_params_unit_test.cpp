#include <gtest/gtest.h>

#include "local_planner/skills/skill_util.hpp"
#include "local_planner/skills/task.hpp"
#include "../helper/id_provider.hpp"

#include "config_provider/config_store_main.hpp"

namespace luhsoccer::local_planner {

TEST(ComponentParam, BoolConstructionAndAccess) {
    auto wm = std::make_shared<transform::WorldModel>();
    TaskData td(tests::IDProvider::createID(0, Team::ALLY));
    BoolComponentParam c1(true);
    EXPECT_TRUE(c1.val(wm, td));

    BoolComponentParam c2(false);
    EXPECT_FALSE(c2.val(wm, td));

    BoolComponentParam p1(config_provider::ConfigProvider::getConfigStore().demo_config.bool_param);
    EXPECT_EQ(p1.val(wm, td), config_provider::ConfigProvider::getConfigStore().demo_config.bool_param.val());

    bool bool_val = false;
    BoolComponentParam cb1(CALLBACK, [&bool_val](const std::shared_ptr<const transform::WorldModel>& wm,
                                                 const TaskData& td) { return bool_val; });
    EXPECT_FALSE(cb1.val(wm, td));
    bool_val = true;
    EXPECT_TRUE(cb1.val(wm, td));

    BoolComponentParam cb2(
        CALLBACK, [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) { return true; });
    EXPECT_TRUE(cb2.val(wm, td));

    td.required_bools = {false};
    BoolComponentParam tdb(TD, 0);
    EXPECT_FALSE(tdb.val(wm, td));
    td.required_bools = {true};
    EXPECT_TRUE(tdb.val(wm, td));

    DoubleComponentParam cb3(CALLBACK, [](const CallbackData& data) -> double {
        auto a = data.td.getCookie<double>(data.component_uid, "data");
        if (a.has_value()) {
            return a.value();
        } else {
            data.td.setCookie(data.component_uid, "data", 3.0);
            return 2.0;
        }
    });
    EXPECT_EQ(cb3.val(wm, td), 2.0);
    EXPECT_EQ(cb3.val(wm, td), 3.0);
    auto cb4 = cb3;
    EXPECT_EQ(cb4.val(wm, td), 2.0);
    EXPECT_EQ(cb4.val(wm, td), 3.0);
}

}  // namespace luhsoccer::local_planner
