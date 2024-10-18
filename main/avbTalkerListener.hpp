#ifndef COMPONENTS_ATDECC_INCLUDE_ATDECCTALKERLISTENER_HPP_
#define COMPONENTS_ATDECC_INCLUDE_ATDECCTALKERLISTENER_HPP_

#include "protocolAemAecpdu.hpp"
#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <map>
#include <memory>
#include <esp_system.h>
#include <esp_log.h>
#include <esp_err.h>
#include "lwip/sockets.h"
#include "protocolAdpdu.hpp"
#include "protocolAcmpdu.hpp"
#include "entity.hpp"
#include "entityModelTree.hpp"
#include "entityEnums.hpp"
#include "protocolAemPayloads.hpp"
#include "sdkconfig.h"

// Constants
constexpr std::uint16_t AVTP_ETHERTYPE = 0x22f0;  // AVTP Ethertype
constexpr std::uint16_t ATDECC_DEFAULT_PORT = 17221;  // ATDECC default UDP port
constexpr std::uint32_t ENTITY_ID = static_cast<std::uint32_t>(0x001b92fffe01b930);  // Placeholder entity ID
constexpr std::uint64_t ENTITY_MODEL_ID = static_cast<std::uint64_t>(0x123456789ABCDEF);  // Placeholder entity ID
constexpr std::uint16_t ENTITY_CAPABILITIES = static_cast<std::uint16_t>(0x123456);
constexpr std::uint32_t INTERFACE_VERSION = 304;

// Enumeration for ATDECC states
enum class AtdeccState : std::uint8_t
{
    UNINITIALIZED,
    DISCOVERED,
    CONFIGURED,
    CONNECTED,
    DISCONNECTED
};

class AemHandler final
{
public:
	AemHandler(Entity const& entity, EntityTree const* const entityModelTree);

	static void validateEntityModel(EntityTree const* const entityModelTree);

	bool onUnhandledAecpAemCommand(AemAecpdu const& aem) const noexcept;

	// Deleted compiler auto-generated methods
	AemHandler(AemHandler const&) = delete;
	AemHandler(AemHandler&&) = delete;
	AemHandler& operator=(AemHandler const&) = delete;
	AemHandler& operator=(AemHandler&&) = delete;

private:
	EntityDescriptor buildEntityDescriptor() const noexcept;
	ConfigurationDescriptor buildConfigurationDescriptor(ConfigurationIndex const configIndex) const;

	Entity const& _entity;
	EntityTree const* _entityModelTree{ nullptr };
};

// ATDECC Talker and Listener class
class AtdeccTalkerListener
{
public:
    // Constructor and Destructor
    Entity entity_;
    AtdeccTalkerListener(Entity const& entity, EntityTree const* entityModelTree);
    ~AtdeccTalkerListener();

    // Initialization
    esp_err_t initialize();

    // ATDECC Operations
    esp_err_t discover();
    esp_err_t configure();
    esp_err_t connect();
    esp_err_t disconnect();

    // Get Current State
    AtdeccState getState() const;
    
    // Method to create Entity Available ADP message
    void sendEntityAvailable(uint64_t entityID, uint64_t entityModelID, uint32_t entityCapabilities);

    // Method to create Entity Departing ADP message
    void sendEntityDeparting(uint64_t entityID, uint32_t availableIndex);

    // Method to create Entity Discover ADP message
    void sendEntityDiscover(uint64_t entityID);
    
    // Method to send an ACMP (connect stream command)
    void sendConnectStream();

private:
    // Internal State
    AtdeccState state_;
    
    // Networking
    int socket_fd_;
    struct sockaddr_in local_addr_;
    struct sockaddr_in remote_addr_;

    // AemHandler instance
    AemHandler aemHandler_;  // <-- New instance of AemHandler

    // Helper Methods
    esp_err_t createSocket();
    esp_err_t bindSocket();
    esp_err_t receivePacket(std::vector<uint8_t>& packet);
    esp_err_t sendPacket(const std::vector<uint8_t>& packet);

    // Logging Tag
    static constexpr const char* TAG = "ATDECC_TALKER_LISTENER";
    
    // Handle ADP message
    void handleAdpMessage(const Adpdu& adpMessage);

    // Send ADP message
    void sendAdpMessage(const Adpdu& adpMessage);
    void receiveAndHandleAdpMessage();
    
    // Send ACMP message
    void sendAcmpMessage(const Acmpdu& acmpMessage);
    
    // Handle AEM commands
    void handleAemMessage();
    
    void handleAcquireEntity(const AemAecpdu& aemMessage);
    void handleLockEntity(const AemAecpdu& aemMessage);
    void handleSetConfiguration(const AemAecpdu& aemMessage);
    void handleGetConfiguration(const AemAecpdu& aemMessage);
    void handleSetStreamFormat(const AemAecpdu& aemMessage);
    void handleGetStreamFormat(const AemAecpdu& aemMessage);
    void handleSetName(const AemAecpdu& aemMessage);
    void handleGetName(const AemAecpdu& aemMessage);
    void sendAemResponse(const AemAecpdu& response);
};

void sendAemResponse(const AemAecpdu& response);

#endif /* COMPONENTS_ATDECC_INCLUDE_ATDECCTALKERLISTENER_HPP_ */
