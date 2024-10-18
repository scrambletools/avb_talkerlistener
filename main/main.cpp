#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_eth.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "uniqueIdentifier.hpp"
#include "entity.hpp"
#include "entityModel.hpp"
#include "entityModelTree.hpp"
#include "protocolAdpdu.hpp"
#include "avbTalkerListener.hpp"

// Wait for tester to start monitoring serial port
static bool monitor_wait = true;
// Define logging tag
static const char *TAG = "avb-tl";
//static auto const s_TalkerEntityID = UniqueIdentifier{0x1b92fffe02233b};
static bool ip_obtained = false;  // Global flag for IP status

// Global Ethernet handle
static esp_eth_handle_t eth_handle = NULL;

// Ethernet event handler
static void eth_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
	esp_netif_t *eth_netif = (esp_netif_t *)arg;
	
    if (event_id == ETHERNET_EVENT_CONNECTED) {

        if (monitor_wait) {
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
        ESP_LOGI(TAG, "Ethernet Link Up");
        
        vTaskDelay(pdMS_TO_TICKS(500)); // Delay to ensure link is stable

        /* Attempt to get IP address */
        esp_netif_ip_info_t ip_info;
        if (esp_netif_get_ip_info(eth_netif, &ip_info) == ESP_OK) {
            if (ip_info.ip.addr != 0) {
                ESP_LOGI(TAG, "IP Address: " IPSTR, IP2STR(&ip_info.ip));
            } else {
                ESP_LOGI(TAG, "IP Address not assigned yet.");
            }
        } else {
            ESP_LOGE(TAG, "Failed to get IP address");
        }
    } else if (event_id == ETHERNET_EVENT_DISCONNECTED) {
        ESP_LOGI(TAG, "Ethernet Link Down");
    } else if (event_id == ETHERNET_EVENT_START) {
        ESP_LOGI(TAG, "Ethernet Started");
    } else if (event_id == ETHERNET_EVENT_STOP) {
        ESP_LOGI(TAG, "Ethernet Stopped");
    }
}

static void got_ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    
    // Set the IP obtained flag to true
    ip_obtained = true;
}

// Ethernet initialization function for ESP32 with LAN8720 PHY
void init_ethernet()
{
    // Initialize TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_init());

    // Create default event loop if not already created
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Create netif for Ethernet
    esp_netif_config_t netif_config = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&netif_config);

    // Configure GPIO16 to enable the oscillator
    gpio_reset_pin(GPIO_NUM_16);
    gpio_set_direction(GPIO_NUM_16, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_16, 1); // Set GPIO16 high to enable the oscillator

    // Configure default Ethernet MAC and PHY settings
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_esp32_emac_config_t esp32_emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();

    // Configure the SMI (MDC and MDIO) GPIO pins
    esp32_emac_config.smi_gpio.mdc_num = GPIO_NUM_23;  // MDC pin
    esp32_emac_config.smi_gpio.mdio_num = GPIO_NUM_18; // MDIO pin

    // Create MAC instance using the internal ESP32 EMAC
    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&esp32_emac_config, &mac_config);

    // Configure PHY settings
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = 1;  // Use the default PHY address
    phy_config.reset_gpio_num = GPIO_NUM_5;
    
    gpio_set_direction(GPIO_NUM_5, GPIO_MODE_OUTPUT);
	gpio_set_level(GPIO_NUM_5, 0); // Set low to reset
	vTaskDelay(pdMS_TO_TICKS(100)); // Delay for PHY reset
	gpio_set_level(GPIO_NUM_5, 1); // Set high to take PHY out of reset

    // Create PHY instance for LAN8720
    esp_eth_phy_t *phy = esp_eth_phy_new_lan87xx(&phy_config);

    // Ethernet configuration linking MAC and PHY
    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);

    // Install Ethernet driver
    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config, &eth_handle));

    // Attach Ethernet driver to TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));

    // Register the event handler for Ethernet events, passing the netif as argument
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, eth_netif));

    // Register IP event handler
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));

    // Start the Ethernet driver
    ESP_ERROR_CHECK(esp_eth_start(eth_handle));
}

extern "C" void app_main()
{
    // Initialize NVS (non-volatile storage)
    ESP_ERROR_CHECK(nvs_flash_init());

    // Initialize Ethernet and establish a connection
    init_ethernet();
    
    // Main loop
    while (true) {
        if (ip_obtained) {
            ESP_LOGI(TAG, "IP obtained, initializing AtdeccTalkerListener...");

            // Initialize AtdeccTalkerListener only after IP is obtained
            Entity::CommonInformation commonInfo{
			    /*entityID=*/UniqueIdentifier(ENTITY_ID),
			    /*entityModelID=*/UniqueIdentifier(ENTITY_MODEL_ID),
			    /*entityCapabilities=*/EntityCapabilities(EntityCapability::AemSupported),
			    /*talkerStreamSources=*/2,
			    /*talkerCapabilities=*/TalkerCapabilities(TalkerCapability::AudioSource),
			    /*listenerStreamSinks=*/1,
			    /*listenerCapabilities=*/ListenerCapabilities(ListenerCapability::AudioSink),
			    /*controllerCapabilities=*/ControllerCapabilities(ControllerCapability::Implemented),
			    /*identifyControlIndex=*/0,
			    /*associationID=*/UniqueIdentifier(0x1122334455667788)
			};
			
			Entity::InterfaceInformation interfaceInfo{
				{0x00, 0x1A, 0xB6, 0x00, 0x02, 0x03}, /*validTime=*/2, /*availableIndex=*/0, 
			        /*gptpGrandmasterID=*/UniqueIdentifier(0xFFEEDDCCBBAA9988), /*gptpDomainNumber=*/0
			};
			
			// Create the Entity instance
			Entity entity{commonInfo, interfaceInfo};
			
			// Example EntityTree setup
			EntityTree entityModelTree;
			
			// Set up entityModelTree as required...
			// (Fill out entityModelTree's dynamicModel, staticModel, configurationTrees, etc.)
			
			// Instantiate the AtdeccTalkerListener
			AtdeccTalkerListener talkerListener{entity, &entityModelTree};
			
            esp_err_t err = talkerListener.initialize();
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to initialize AtdeccTalkerListener: %s", esp_err_to_name(err));
            } else {
				while (true) {

	                // Send Entity Available message
	                talkerListener.sendEntityAvailable(
	                    commonInfo.entityID.getValue(),
	                    commonInfo.entityModelID.getValue(),
	                    commonInfo.entityCapabilities.getValue()
	                );
	                ESP_LOGI(TAG, "Entity Available message sent");
	
	                // Send Entity Departing message (for example when shutting down)
	                talkerListener.sendEntityDeparting(
	                    commonInfo.entityID.getValue(),
	                    0  // Available index
	                );
	                ESP_LOGI(TAG, "Entity Departing message sent");
	
	                // Send Entity Discover message
	                talkerListener.sendEntityDiscover(commonInfo.entityID.getValue());
	                ESP_LOGI(TAG, "Entity Discover message sent");
	                
	                // Send Entity Available message
	                talkerListener.sendEntityAvailable(
	                    commonInfo.entityID.getValue(),
	                    commonInfo.entityModelID.getValue(),
	                    commonInfo.entityCapabilities.getValue()
	                );
	                ESP_LOGI(TAG, "Entity Available message sent");
	                
	                vTaskDelay(pdMS_TO_TICKS(10000));
	
	                // Send Entity Departing message (for example when shutting down)
	                talkerListener.sendEntityDeparting(
	                    commonInfo.entityID.getValue(),
	                    0  // Available index
	                );
	                ESP_LOGI(TAG, "Entity Departing message sent");
	                
	                vTaskDelay(pdMS_TO_TICKS(10000));
	
	                // Send Entity Discover message
	                talkerListener.sendEntityDiscover(commonInfo.entityID.getValue());
	                ESP_LOGI(TAG, "Entity Discover message sent");
	                
	                vTaskDelay(pdMS_TO_TICKS(10000));
	
	                // Send Connect Stream command (example of ACMP operation)
	                talkerListener.sendConnectStream();
	                ESP_LOGI(TAG, "Connect Stream command sent");
	
	                vTaskDelay(pdMS_TO_TICKS(10000));
	
	                // Send Connect Stream command (example of ACMP operation)
	                talkerListener.sendConnectStream();
	                ESP_LOGI(TAG, "Connect Stream command sent");
	
	                vTaskDelay(pdMS_TO_TICKS(10000));
				}
                
            }
            
            // Break the loop after initialization to avoid redundant initializations
            break;
        }

        // Perform other periodic tasks while waiting for IP (if needed)
        ESP_LOGI(TAG, "Waiting for IP...");
        vTaskDelay(pdMS_TO_TICKS(1000));  // Delay for 1 second before checking again
    }

    // Main loop (optional)
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Polling interval
    }
}
