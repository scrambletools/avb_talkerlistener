#include "avbTalkerListener.hpp"
#include "protocolAcmpdu.hpp"
#include "protocolAemAecpdu.hpp"
#include "protocolAemPayloads.hpp"
#include "protocolDefines.hpp"
#include <lwip/err.h>
#include <lwip/netdb.h>
#include <lwip/sockets.h>

AemHandler::AemHandler(Entity const& entity, EntityTree const* const entityModelTree)
    : _entity{ entity }
    , _entityModelTree{ entityModelTree }
{
    // Validate the entity model
    validateEntityModel(_entityModelTree);
}

void AemHandler::validateEntityModel(EntityTree const* const entityModelTree)
{
    // Briefly validate entity model
    if (entityModelTree != nullptr)
    {
        // Check there is at least one configuration descriptor
        auto const countConfigs = static_cast<DescriptorIndex>(entityModelTree->configurationTrees.size());
        if (countConfigs == 0)
        {
            ESP_LOGE("AEM-HANDLER", "Invalid Entity Model: At least one ConfigurationDescriptor is required");
            return;
        }

        // Check the current configuration index is in the correct range
        if (entityModelTree->dynamicModel.currentConfiguration >= countConfigs)
        {
            ESP_LOGE("AEM-HANDLER", "Invalid Entity Model: Current Configuration Index is out of range");
            return;
        }
    }
}

bool AemHandler::onUnhandledAecpAemCommand(AemAecpdu const& aem) const noexcept
{
	Serializer<AemAecpdu::MAXIMUM_SEND_PAYLOAD_BUFFER_LENGTH> ser;
    switch (aem.getCommandType())
    {
        case AemCommandType::READ_DESCRIPTOR:
        {
            if (_entityModelTree != nullptr)
            {
                auto const [configIndex, descriptorType, descriptorIndex] = deserializeReadDescriptorCommand(aem.getPayload());
                switch (descriptorType)
                {
                    case DescriptorType::Entity:
                        if (configIndex != DescriptorIndex{ 0u } || descriptorIndex != DescriptorIndex{ 0u })
                        {
                            ESP_LOGE("AEM HANDLER", "Bad arguments for Entity Descriptor");
                            return true;
                        }
                        ser = serializeReadDescriptorCommonResponse(configIndex, descriptorType, descriptorIndex);
                        serializeReadEntityDescriptorResponse(ser, buildEntityDescriptor());
                        sendAemResponse(ser.data());
                        return true;

                    case DescriptorType::Configuration:
                        if (configIndex != DescriptorIndex{ 0u })
                        {
                            ESP_LOGE("AEM HANDLER", "Bad arguments for Configuration Descriptor");
                            return true;
                        }
                        ser = serializeReadDescriptorCommonResponse(configIndex, descriptorType, descriptorIndex);
                        serializeReadConfigurationDescriptorResponse(ser, buildConfigurationDescriptor(descriptorIndex));
                        sendAemResponse(ser.data());
                        return true;

                    default:
                        ESP_LOGW("AEM HANDLER", "Unhandled descriptor type in ReadDescriptor");
                        break;
                }
            }
            return false;
        }
        default:
            ESP_LOGW("AEM HANDLER", "Unhandled AEM command type");
            return false;
    }
}

EntityDescriptor AemHandler::buildEntityDescriptor() const noexcept
{
	auto entityDescriptor = EntityDescriptor{};
	Entity::CommonInformation common_info = _entity.getCommonInformation();

	entityDescriptor.entityID = common_info.entityID;
	entityDescriptor.entityModelID = common_info.entityModelID;
	entityDescriptor.entityCapabilities = common_info.entityCapabilities;
	entityDescriptor.talkerStreamSources = common_info.talkerStreamSources;
	entityDescriptor.talkerCapabilities = common_info.talkerCapabilities;
	entityDescriptor.listenerStreamSinks = common_info.listenerStreamSinks;
	entityDescriptor.listenerCapabilities = common_info.listenerCapabilities;
	entityDescriptor.controllerCapabilities = common_info.controllerCapabilities;
	entityDescriptor.availableIndex = 0u;
	if (auto const id = common_info.associationID; id->isValid())
	{
		entityDescriptor.associationID.setValue(common_info.associationID->getValue());
	}
	entityDescriptor.entityName = _entityModelTree->dynamicModel.entityName;
	entityDescriptor.vendorNameString = _entityModelTree->staticModel.vendorNameString;
	entityDescriptor.modelNameString = _entityModelTree->staticModel.modelNameString;
	entityDescriptor.firmwareVersion = _entityModelTree->dynamicModel.firmwareVersion;
	entityDescriptor.groupName = _entityModelTree->dynamicModel.groupName;
	entityDescriptor.serialNumber = _entityModelTree->dynamicModel.serialNumber;
	entityDescriptor.configurationsCount = static_cast<decltype(entityDescriptor.configurationsCount)>(_entityModelTree->configurationTrees.size());
	entityDescriptor.currentConfiguration = _entityModelTree->dynamicModel.currentConfiguration;

	return entityDescriptor;
}

template<DescriptorType DescriptorT, class Tree>
void setDescriptorsCount(ConfigurationDescriptor& configDescriptor, Tree const& tree)
{
	if (!tree.empty())
	{
		configDescriptor.descriptorCounts[DescriptorT] = static_cast<typename decltype(configDescriptor.descriptorCounts)::mapped_type>(tree.size());
	}
}

ConfigurationDescriptor AemHandler::buildConfigurationDescriptor(ConfigurationIndex const configIndex) const
{
	auto configDescriptor = ConfigurationDescriptor{};

	auto const configIt = _entityModelTree->configurationTrees.find(configIndex);
	if (configIt == _entityModelTree->configurationTrees.end())
	{
		ESP_LOGE("AEM-HANDLER", "No Descriptor");
	}

	auto const& configTree = configIt->second;

	configDescriptor.objectName = configTree.dynamicModel.objectName;
	configDescriptor.localizedDescription = configTree.staticModel.localizedDescription;

	setDescriptorsCount<DescriptorType::AudioUnit>(configDescriptor, configTree.audioUnitModels);
	setDescriptorsCount<DescriptorType::StreamInput>(configDescriptor, configTree.streamInputModels);
	setDescriptorsCount<DescriptorType::StreamOutput>(configDescriptor, configTree.streamOutputModels);
	//setDescriptorsCount<DescriptorType::JackInput>(configDescriptor, configTree.jackInputModels);
	//setDescriptorsCount<DescriptorType::JackOutput>(configDescriptor, configTree.jackOutputModels);
	setDescriptorsCount<DescriptorType::AvbInterface>(configDescriptor, configTree.avbInterfaceModels);
	setDescriptorsCount<DescriptorType::ClockSource>(configDescriptor, configTree.clockSourceModels);
	setDescriptorsCount<DescriptorType::MemoryObject>(configDescriptor, configTree.memoryObjectModels);
	setDescriptorsCount<DescriptorType::Locale>(configDescriptor, configTree.localeModels);
	setDescriptorsCount<DescriptorType::Strings>(configDescriptor, configTree.stringsModels);
	setDescriptorsCount<DescriptorType::StreamPortInput>(configDescriptor, configTree.streamPortInputModels);
	setDescriptorsCount<DescriptorType::StreamPortOutput>(configDescriptor, configTree.streamPortOutputModels);
	//setDescriptorsCount<DescriptorType::ExternalPortInput>(configDescriptor, configTree.externalPortInputModels);
	//setDescriptorsCount<DescriptorType::ExternalPortOutput>(configDescriptor, configTree.externalPortOutputModels);
	//setDescriptorsCount<DescriptorType::InternalPortInput>(configDescriptor, configTree.internalPortInputModels);
	//setDescriptorsCount<DescriptorType::InternalPortOutput>(configDescriptor, configTree.internalPortOutputModels);
	setDescriptorsCount<DescriptorType::AudioCluster>(configDescriptor, configTree.audioClusterModels);
	setDescriptorsCount<DescriptorType::AudioMap>(configDescriptor, configTree.audioMapModels);
	setDescriptorsCount<DescriptorType::Control>(configDescriptor, configTree.controlModels);
	setDescriptorsCount<DescriptorType::ClockDomain>(configDescriptor, configTree.clockDomainModels);

	std::unordered_map<DescriptorType, std::uint16_t, EnumClassHash> descriptorCounts{};

	return configDescriptor;
}

// Constructor
AtdeccTalkerListener::AtdeccTalkerListener(Entity const& entity, EntityTree const* entityModelTree)
    : entity_(entity),
      state_(AtdeccState::UNINITIALIZED),  // Initialize AemHandler
      socket_fd_(-1),
      aemHandler_(entity, entityModelTree)
{
    memset(&local_addr_, 0, sizeof(local_addr_));
    memset(&remote_addr_, 0, sizeof(remote_addr_));
}

// Destructor
AtdeccTalkerListener::~AtdeccTalkerListener()
{
    if (socket_fd_ >= 0)
    {
        close(socket_fd_);
    }
}

// Initialize the Talker/Listener for Broadcast
esp_err_t AtdeccTalkerListener::initialize()
{
    // Create a UDP socket
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);  // Use SOCK_DGRAM for UDP
    if (socket_fd_ < 0) {
        ESP_LOGE(TAG, "Failed to create socket: errno %d", errno);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Socket created successfully");

    // Enable broadcasting on the socket
    int broadcast_enable = 1;
    if (setsockopt(socket_fd_, SOL_SOCKET, SO_BROADCAST, &broadcast_enable, sizeof(broadcast_enable)) < 0) {
        ESP_LOGE(TAG, "Failed to enable broadcast: errno %d", errno);
        close(socket_fd_);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Broadcast enabled on socket");

    // Set up the local address to bind to
    struct sockaddr_in local_addr;
    local_addr.sin_family = AF_INET;
    local_addr.sin_port = htons(ATDECC_DEFAULT_PORT);  // Bind to the desired port
    local_addr.sin_addr.s_addr = INADDR_ANY;  // Bind to any local IP address

    // Bind the socket to the local address and port
    if (bind(socket_fd_, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
        ESP_LOGE(TAG, "Socket bind failed: errno %d", errno);
        close(socket_fd_);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Socket bound to port %d", ATDECC_DEFAULT_PORT);

    // Set the remote address for broadcasting
    memset(&remote_addr_, 0, sizeof(remote_addr_));
    remote_addr_.sin_family = AF_INET;
    remote_addr_.sin_port = htons(ATDECC_DEFAULT_PORT);  // Set remote port
    remote_addr_.sin_addr.s_addr = inet_addr("192.168.8.255");  // Broadcast address (can change for subnet-specific)

    ESP_LOGI(TAG, "Broadcasting to port %d", ATDECC_DEFAULT_PORT);

    // Set initial state
    state_ = AtdeccState::DISCOVERED;
    return ESP_OK;
}

// Discover the ATDECC entity
esp_err_t AtdeccTalkerListener::discover()
{
    if (state_ != AtdeccState::UNINITIALIZED)
    {
        Adpdu adpMessage;
        sendEntityDiscover(ENTITY_ID);

        state_ = AtdeccState::DISCOVERED;
        ESP_LOGI(TAG, "Entity discovered");
        return ESP_OK;
    }
    return ESP_ERR_INVALID_STATE;
}

// Configure the ATDECC entity
esp_err_t AtdeccTalkerListener::configure()
{
    if (state_ == AtdeccState::DISCOVERED)
    {
        // Perform configuration operations here
        state_ = AtdeccState::CONFIGURED;
        ESP_LOGI(TAG, "Entity configured");
        return ESP_OK;
    }
    return ESP_ERR_INVALID_STATE;
}

// Connect the ATDECC entity
esp_err_t AtdeccTalkerListener::connect()
{
    if (state_ == AtdeccState::CONFIGURED)
    {
        // Handle connection setup
        state_ = AtdeccState::CONNECTED;
        ESP_LOGI(TAG, "Entity connected");
        return ESP_OK;
    }
    return ESP_ERR_INVALID_STATE;
}

// Disconnect the ATDECC entity
esp_err_t AtdeccTalkerListener::disconnect()
{
    if (state_ == AtdeccState::CONNECTED)
    {
        // Handle disconnection
        state_ = AtdeccState::DISCONNECTED;
        ESP_LOGI(TAG, "Entity disconnected");
        return ESP_OK;
    }
    return ESP_ERR_INVALID_STATE;
}

// Get the current state
AtdeccState AtdeccTalkerListener::getState() const
{
    return state_;
}

// Create and configure the socket
esp_err_t AtdeccTalkerListener::createSocket()
{
    // Create UDP socket
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socket_fd_ < 0)
    {
        ESP_LOGE(TAG, "Failed to create socket");
        return ESP_FAIL;
    }

    // Configure local address
    local_addr_.sin_family = AF_INET;
    local_addr_.sin_addr.s_addr = htonl(INADDR_ANY);
    local_addr_.sin_port = htons(ATDECC_DEFAULT_PORT);
    return ESP_OK;
}

// Bind the socket to the local address
esp_err_t AtdeccTalkerListener::bindSocket()
{
    if (bind(socket_fd_, (struct sockaddr*)&local_addr_, sizeof(local_addr_)) < 0)
    {
        ESP_LOGE(TAG, "Failed to bind socket");
        close(socket_fd_);
        return ESP_FAIL;
    }
    return ESP_OK;
}

// Receive a packet
esp_err_t AtdeccTalkerListener::receivePacket(std::vector<uint8_t>& packet)
{
    uint8_t buffer[1024];
    socklen_t addr_len = sizeof(remote_addr_);
    int len = recvfrom(socket_fd_, buffer, sizeof(buffer), 0, (struct sockaddr*)&remote_addr_, &addr_len);
    if (len < 0)
    {
        ESP_LOGE(TAG, "Failed to receive packet");
        return ESP_FAIL;
    }

    packet.assign(buffer, buffer + len);
    return ESP_OK;
}

// Send a packet
esp_err_t AtdeccTalkerListener::sendPacket(const std::vector<uint8_t>& packet)
{
    int len = sendto(socket_fd_, packet.data(), packet.size(), 0, (struct sockaddr*)&remote_addr_, sizeof(remote_addr_));
	if (len < 0)
	{
	    ESP_LOGE(TAG, "Failed to send packet: errno %d", errno);
	    return ESP_FAIL;
	}
    return ESP_OK;
}

// Handle received ADP message
void AtdeccTalkerListener::handleAdpMessage(const Adpdu& adpMessage)
{
    switch (static_cast<AdpMessageType>(adpMessage.getMessageType()))
    {
        case AdpMessageType::ENTITY_DISCOVER:
            ESP_LOGI(TAG, "Received Entity Discover message.");
            if (state_ == AtdeccState::DISCOVERED || state_ == AtdeccState::CONFIGURED)
            {
                sendEntityAvailable(ENTITY_ID, ENTITY_MODEL_ID, ENTITY_CAPABILITIES);
            }
            break;

        case AdpMessageType::ENTITY_AVAILABLE:
            ESP_LOGI(TAG, "Received Entity Available message. Entity ID: 0x%llx", adpMessage.getEntityID());
            // Process the Entity Available message
            break;

        case AdpMessageType::ENTITY_DEPARTING:
            ESP_LOGI(TAG, "Received Entity Departing message. Entity ID: 0x%llx", adpMessage.getEntityID());
            // Remove the entity from the list of known entities
            break;

        default:
            ESP_LOGW(TAG, "Received unknown ADP message type.");
            break;
    }
}

// Print buffer in hexadecimal format
void printBuffer(const std::vector<uint8_t>& buffer)
{
    ESP_LOGI("BUFFER", "Buffer contents: ");
    for (size_t i = 0; i < buffer.size(); ++i) {
        ESP_LOGI("BUFFER", "%02X", buffer[i]);  // Print each byte as a 2-digit hexadecimal number
    }
}

// Send an ADP message over the network
void AtdeccTalkerListener::sendAdpMessage(const Adpdu& adpMessage)
{
    std::vector<uint8_t> buffer(Adpdu::Length);
    adpMessage.serialize(buffer.data()); // Serialize the ADPDU to the buffer
    //printBuffer(buffer);
    sendPacket(buffer); // Send the serialized buffer over the network
}

// Send an ADP message over the network
void AtdeccTalkerListener::sendAcmpMessage(const Acmpdu& acmpMessage)
{
    std::vector<uint8_t> buffer(Adpdu::Length);
    acmpMessage.serialize(buffer.data()); // Serialize the ADPDU to the buffer
    //printBuffer(buffer);
    sendPacket(buffer); // Send the serialized buffer over the network
}

// Handle incoming ADPDU messages by deserializing and processing
void AtdeccTalkerListener::receiveAndHandleAdpMessage()
{
    std::vector<uint8_t> buffer;
    if (receivePacket(buffer) == ESP_OK)
    {
        Adpdu adpMessage;
        adpMessage.deserialize(buffer.data()); // Deserialize buffer into an ADPDU object
        handleAdpMessage(adpMessage); // Handle the ADP message accordingly
    }
}

// Send Entity Available message using the Adpdu class
void AtdeccTalkerListener::sendEntityAvailable(uint64_t entityID, uint64_t entityModelID, uint32_t entityCapabilities)
{
    Adpdu adpMessage = Adpdu::createEntityAvailableMessage(entityID, entityModelID, entityCapabilities);
    /*ESP_LOGI("ADP", "Creating Entity Available message");
    ESP_LOGI("ADP", "Message Type: 0x%08X", (unsigned char)adpMessage.messageType); 
    ESP_LOGI("ADP", "Entity ID: 0x%016llX", adpMessage.entityID);              // Print Entity ID
    ESP_LOGI("ADP", "Entity Model ID: 0x%016llX", adpMessage.entityModelID);    // Print Entity Model ID
    ESP_LOGI("ADP", "Entity Capabilities: 0x%08X", (unsigned int)adpMessage.entityCapabilities);  // Print Entity Capabilities*/
    sendAdpMessage(adpMessage);
}

// Send Entity Departing message using the Adpdu class
void AtdeccTalkerListener::sendEntityDeparting(uint64_t entityID, uint32_t availableIndex)
{
    Adpdu adpMessage = Adpdu::createEntityDepartingMessage(entityID, availableIndex);
    sendAdpMessage(adpMessage);
}

// Send Entity Discover message using the Adpdu class
void AtdeccTalkerListener::sendEntityDiscover(uint64_t entityID)
{
    Adpdu adpMessage = Adpdu::createEntityDiscoverMessage();
    sendAdpMessage(adpMessage);
}

// Send Connect Stream command via the ACMP protocol
void AtdeccTalkerListener::sendConnectStream()
{
	Acmpdu acmpMessage = Acmpdu::createConnectTxCommand();
	sendAcmpMessage(acmpMessage);
}

// Helper function to send AEM responses
void AtdeccTalkerListener::sendAemResponse(const AemAecpdu& response)
{
    std::vector<uint8_t> buffer(AemAecpdu::MAXIMUM_SEND_PAYLOAD_BUFFER_LENGTH);
    response.serialize(buffer.data(), buffer.size()); // Serialize response into buffer
    sendPacket(buffer); // Send the serialized buffer over the network
}

void AtdeccTalkerListener::handleAemMessage()
{
    std::vector<uint8_t> buffer;
    if (receivePacket(buffer) == ESP_OK)
    {
        AemAecpdu aemMessage(0);
        aemMessage.deserialize(buffer.data(), buffer.size()); // Deserialize the buffer into an AEM message
        
        if (!aemHandler_.onUnhandledAecpAemCommand(aemMessage))
        {
            // If AemHandler cannot handle the message, handle specific commands locally
            switch (static_cast<AemCommandType>(aemMessage.getCommandType()))
            {
                case AemCommandType::ACQUIRE_ENTITY:
                    handleAcquireEntity(aemMessage);
                    break;
                case AemCommandType::LOCK_ENTITY:
                    handleLockEntity(aemMessage);
                    break;
                case AemCommandType::SET_CONFIGURATION:
                    handleSetConfiguration(aemMessage);
                    break;
                case AemCommandType::GET_CONFIGURATION:
                    handleGetConfiguration(aemMessage);
                    break;
                case AemCommandType::SET_STREAM_FORMAT:
                    handleSetStreamFormat(aemMessage);
                    break;
                case AemCommandType::GET_STREAM_FORMAT:
                    handleGetStreamFormat(aemMessage);
                    break;
                case AemCommandType::SET_NAME:
                    handleSetName(aemMessage);
                    break;
                case AemCommandType::GET_NAME:
                    handleGetName(aemMessage);
                    break;
                default:
                    ESP_LOGW(TAG, "Unhandled AEM command");
                    break;
            }
        }
    }
}

// Helper function to handle ACQUIRE_ENTITY command
void AtdeccTalkerListener::handleAcquireEntity(const AemAecpdu& aemMessage)
{
	Entity::CommonInformation common_info = entity_.getCommonInformation();
    if (common_info.entityCapabilities.getValue() == static_cast<uint32_t>(EntityCapability::AemSupported))
    {
        ESP_LOGI(TAG, "Processing Acquire Entity command.");
        auto [flags, ownerID, descriptorType, descriptorIndex] = deserializeAcquireEntityCommand(aemMessage.getPayload());
        
        // Set the entity as acquired
        if (static_cast<uint32_t>(flags) & static_cast<uint32_t>(AemAcquireEntityFlags::NONE))
        {
            // Get the current capabilities
			EntityCapabilities entityCapabilities = common_info.entityCapabilities;
			
			// Set the EntityNotReady flag
			entityCapabilities.setFlag(EntityCapability::EntityNotReady);
			
			// Update the entity's capabilities using the EnumBitfield
			entity_.setEntityCapabilities(entityCapabilities);
            ESP_LOGI(TAG, "Entity is acquired by owner %lu.", static_cast<uint32_t>(ownerID));
        }
    }
    else
    {
        ESP_LOGW(TAG, "Entity does not support AEM. Ignoring Acquire Entity.");
    }
}

// Helper function to handle LOCK_ENTITY command
void AtdeccTalkerListener::handleLockEntity(const AemAecpdu& aemMessage)
{
    ESP_LOGI(TAG, "Processing Lock Entity command.");
    auto [flags, lockedID, descriptorType, descriptorIndex] = deserializeLockEntityCommand(aemMessage.getPayload());
    
    Entity::CommonInformation common_info = entity_.getCommonInformation();

    if (static_cast<uint32_t>(flags) & static_cast<uint32_t>(AemLockEntityFlags::NONE))
    {
        ESP_LOGI(TAG, "Entity locked by controller %lu.", static_cast<uint32_t>(lockedID));
        // Set the EntityNotReady flag
        common_info.entityCapabilities.setFlag(EntityCapability::EntityNotReady);
        entity_.setEntityCapabilities(common_info.entityCapabilities);
    }
    else if (static_cast<uint32_t>(flags) & static_cast<uint32_t>(AemLockEntityFlags::UNLOCK))
    {
        ESP_LOGI(TAG, "Entity unlocked.");
        // Remove the EntityNotReady flag
        common_info.entityCapabilities.removeFlag(EntityCapability::EntityNotReady);
        entity_.setEntityCapabilities(common_info.entityCapabilities);
    }
}


/*// Helper function to handle SET_CONFIGURATION command
void AtdeccTalkerListener::handleSetConfiguration(const AemAecpdu& aemMessage)
{
    ESP_LOGI(TAG, "Processing Set Configuration command.");
    auto [configurationIndex] = deserializeSetConfigurationCommand(aemMessage.getPayload());
    
    // Set the configuration for the entity
    // Example: set the configuration for a stream or descriptor
    ESP_LOGI(TAG, "Setting configuration index %d.", configurationIndex);
    // Entity-specific logic for setting configuration goes here
}

// Helper function to handle GET_CONFIGURATION command
void AtdeccTalkerListener::handleGetConfiguration(const AemAecpdu& aemMessage)
{
    ESP_LOGI(TAG, "Processing Get Configuration command.");
    
    uint16_t currentConfiguration = 0; // Retrieve current configuration from the entity
    AemAecpdu response = serializeGetConfigurationResponse(static_cast<uint32_t>(currentConfiguration)).data();
    
    sendAemResponse(response);
}

// Helper function to handle SET_STREAM_FORMAT command
void AtdeccTalkerListener::handleSetStreamFormat(const AemAecpdu& aemMessage)
{
    ESP_LOGI(TAG, "Processing Set Stream Format command.");
    auto [descriptorType, descriptorIndex, streamFormat] = deserializeSetStreamFormatCommand(aemMessage.getPayload());

    if (entity_.setEntityCapabilities().test(EntityCapability::ClassASupported))
    {
        // Set the stream format for the entity
        ESP_LOGI(TAG, "Stream format set for descriptor %d.", descriptorIndex);
        // Apply the new stream format to the entity
    }
    else
    {
        ESP_LOGW(TAG, "Entity does not support Class A Streams.");
    }
}

// Helper function to handle GET_STREAM_FORMAT command
void AtdeccTalkerListener::handleGetStreamFormat(const AemAecpdu& aemMessage)
{
    ESP_LOGI(TAG, "Processing Get Stream Format command.");
    auto [descriptorType, descriptorIndex] = deserializeGetStreamFormatCommand(aemMessage.getPayload());

    // Retrieve the stream format for the entity
    StreamFormat currentStreamFormat = StreamFormat::defaultStreamFormat();
    AemAecpdu response = serializeGetStreamFormatResponse(descriptorType, descriptorIndex, currentStreamFormat).data();
    
    sendAemResponse(response);
}

// Helper function to handle SET_NAME command
void AtdeccTalkerListener::handleSetName(const AemAecpdu& aemMessage)
{
    ESP_LOGI(TAG, "Processing Set Name command.");
    auto [descriptorType, descriptorIndex, nameIndex, configurationIndex, name] = deserializeSetNameCommand(aemMessage.getPayload());

    // Set the name of the entity or descriptor (implementation-specific)
    ESP_LOGI(TAG, "Setting name for descriptor %d.", descriptorIndex);
    // Apply name setting logic here
}

// Helper function to handle GET_NAME command
void AtdeccTalkerListener::handleGetName(const AemAecpdu& aemMessage)
{
    ESP_LOGI(TAG, "Processing Get Name command.");
    auto [descriptorType, descriptorIndex, nameIndex, configurationIndex] = deserializeGetNameCommand(aemMessage.getPayload());

    // Retrieve the name of the entity or descriptor
    AtdeccFixedString name = "EntityName"; // Example name
    AemAecpdu response = serializeGetNameResponse(descriptorType, descriptorIndex, nameIndex, configurationIndex, name);
    
    sendAemResponse(response);
}*/