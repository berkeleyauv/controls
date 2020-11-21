// Example of custom SyncActionNode (synchronous action)
// without ports.
class DetectGate : public BT::SyncActionNode
{
  public:
    DetectGate(const std::string& name) :
        BT::SyncActionNode(name, {})
    {
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override
    {
        std::cout << "Detecting the gate: " << this->name() << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};