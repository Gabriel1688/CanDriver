#include <openarm/canbus/can_device_collection.hpp>
#include <openarm/canbus/can_device.hpp>
#include <openarm/canbus/can_socket.hpp>
CANDeviceCollection::CANDeviceCollection(CANSocket& can_socket) : can_socket_(can_socket) {
    // configure and register to can socket
    client_observer_t observer;
    observer.id = 0x11;
    observer.incomingPacketHandler = std::bind(&CANDeviceCollection::dispatch_frame_callback, this, std::placeholders::_1);
    observer.disconnectionHandler  = nullptr;
    can_socket_.subscribe(observer.id, observer);

    client_observer_t observer1;
    observer1.id = 0x12;
    observer1.incomingPacketHandler = std::bind(&CANDeviceCollection::dispatch_frame_callback, this, std::placeholders::_1);
    observer1.disconnectionHandler  = nullptr;
    can_socket_.subscribe(observer1.id, observer1);

    client_observer_t observer2;
    observer2.id = 0x18;
    observer2.incomingPacketHandler = std::bind(&CANDeviceCollection::dispatch_frame_callback, this, std::placeholders::_1);
    observer2.disconnectionHandler  = nullptr;

    can_socket_.subscribe(observer2.id, observer2);
}

CANDeviceCollection::~CANDeviceCollection() {}

void CANDeviceCollection::add_device(const std::shared_ptr<CANDevice>& device) {
    if (!device) return;

    // Add device to our collection
    canid_t device_id = device->get_recv_can_id();
    devices_[device_id] = device;
}

void CANDeviceCollection::remove_device(const std::shared_ptr<CANDevice>& device) {
    if (!device) return;
    canid_t device_id = device->get_recv_can_id();
    auto it = devices_.find(device_id);
    if (it != devices_.end()) {
        // Remove from our collection
        devices_.erase(it);
    }
}

void CANDeviceCollection::dispatch_frame_callback(const can_frame& frame) {
    auto it = devices_.find(frame.can_id);
    if (it != devices_.end()) {
        it->second->callback(frame);
    }
    // Note: Silently ignore frames for unknown devices (this is normal in CAN networks)
}