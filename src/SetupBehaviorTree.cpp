#include "behaviortree_cpp_v3/bt_factory.h"

int main()
{
    // We use the BehaviorTreeFactory to register our custom nodes
    BehaviorTreeFactory factory;

    // Note: the name used to register should be the same used in the XML.
    using namespace MainNodes;

    // The recommended way to create a Node is through inheritance.
    factory.registerNodeType<DetectGate>("ApproachObject");
    // Trees are created at deployment-time (i.e. at run-time, but only
    // once at the beginning).

    // IMPORTANT: when the object "tree" goes out of scope, all the
    // TreeNodes are destroyed
    auto tree = factory.createTreeFromFile("./tree.xml");

    // To "execute" a Tree you need to "tick" it.
    // The tick is propagated to the children based on the logic of the tree.
    // In this case, the entire sequence is executed, because all the children
    // of the Sequence return SUCCESS.
    tree.tickRoot();

    return 0;
}