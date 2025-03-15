#include "behaviortree_cpp_v3/bt_factory.h"
#include "nodes/route_node.cpp"
#include "nodes/wait_for_pickup_confirmation_condition_node.cpp"
#include "nodes/wait_for_drop_off_confirmation_condition_node.cpp"

using namespace BT;

int main()
{
    BehaviorTreeFactory factory;

    auto tree = factory.createTreeFromText(R"(
<root>
    <sequence>
        <WaitForRoute pickup_pose="{pickup}" dropoff_pose="{dropoff}"/>
        <NavigateToLocation name="{pickup}"/>
        <WaitForPickupConfirmation/>
        <NavigateToLocation name="{dropoff}"/>
        <fallback>
            <DetourHandler detour="{detour}">
                <sequence>
                    <PauseRoute/>
                    <NavigateToLocation name="{detour}"/>
                    <WaitForDetourConfirmation/>
                    <ResumeRoute/>
                </sequence>
            </DetourHandler>
        </fallback>
        <WaitForDropOffConfirmation/>
    </sequence>
</root>
    )");


    factory.registerSimpleCondition("WaitForRoute", std::bind(&WaitForRoute));
    factory.registerSimpleCondition("WaitForPickupConfirmation", std::bind(&WaitForPickupConfirmationFunction));
    factory.registerSimpleCondition("WaitForDropOffConfirmation", std::bind(&WaitForDropOffConfirmationFunction));
    factory.registerNodeType<WaitForRoute>("WaitForRoute");
    factory.registerNodeType<WaitForPickupConfirmation>("WaitForPickupConfirmation");
    factory.registerNodeType<WaitForDropOffConfirmation>("WaitForDropOffConfirmation");

    factory.registerSimpleAction("NavigateToLocation", std::bind(&NavigateToLocationFunction, std::placeholders::_1));
    factory.registerSimpleAction("PauseRoute", std::bind(&PauseRouteFunction));
    factory.registerSimpleAction("ResumeRoute", std::bind(&ResumeRouteFunction));

    while (tree.tickRoot() == NodeStatus::RUNNING)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return 0;
}
