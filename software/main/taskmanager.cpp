#include "taskmanager.h"

// local includes
#include "measurements.h"
#include "mosfets.h"
#include "ledstrip.h"

using namespace std::chrono_literals;

namespace {

using namespace espcpputils;

void noop() {}

SchedulerTask schedulerTaskArr[] {
    SchedulerTask { "measure", measure::init, measure::update, 50ms },
    SchedulerTask { "ledstrip", ledstrip::init, ledstrip::update, 50ms },
    //SchedulerTask { "mosfets", mosfets::init, mosfets::update, 50ms },
};

} // namespace

cpputils::ArrayView<SchedulerTask> schedulerTasks{std::begin(schedulerTaskArr), std::end(schedulerTaskArr)};
