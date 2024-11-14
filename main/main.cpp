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
#include "esp_vfs_l2tap.h"
#include "lwip/prot/ethernet.h" // Ethernet header
#include "driver/timer.h"
#include "uniqueIdentifier.hpp"
#include "entity.hpp"
#include "entityModel.hpp"
#include "entityModelTree.hpp"
#include "protocolAdpdu.hpp"
#include "avbTalkerListener.hpp"
#include "lwip/ip_addr.h"

// static IP fallback values
#define FALLBACK_IP_ADDR            "169.254.18.100"
#define FALLBACK_NETMASK_ADDR       "255.255.0.0"
#define FALLBACK_GW_ADDR            "169.254.18.100"
#define FALLBACK_MAIN_DNS_SERVER    "169.254.18.100"
#define FALLBACK_BACKUP_DNS_SERVER  "169.254.18.1"
#define DHCP_TIMEOUT 10000000
// timer settings
#define TIMER_DIVIDER   (16)  //  Hardware timer clock divider
#define TIMER_SCALE     (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds

//#define IPADDR_TYPE_V4  0

// test device is 48:e7:29:a8:b4:af
// sonet is       00:30:93:19:06:7e
// bcast addr is  91:e0:f0:01:00:00

/* sonet entity avail frame
91 e0 f0 01 00 00 00 30 93 19 06 7e 22 f0 fa 00
40 38 1d 57 dc 6f b4 19 80 00 00 0d 93 00 00 00
00 0c 00 00 85 8a 00 0b 4a 01 00 09 4a 01 00 00
00 00 00 00 06 d2 72 f4 66 af b0 98 00 02 00 00
00 00 00 00 00 00 ff ff ff ff ff ff ff ff 00 00
00 00
*/

// Turn on ethernet frame analyzer
static bool eth_analyzer = true;
// Use IP or not
static bool use_ip = true;
// Wait for tester to start monitoring serial port
static bool startup_wait = true;
// Define logging tag
static const char *TAG = "MAIN";
//static auto const s_TalkerEntityID = UniqueIdentifier{0x1b92fffe02233b};
static bool ip_obtained = false;  // Global flag for IP status
// track dhcp retries
static int dhcp_retry_num = 0;

static esp_err_t set_dns_server(esp_netif_t *netif, uint32_t addr, esp_netif_dns_type_t type)
{
    if (addr && (addr != IPADDR_NONE)) {
        esp_netif_dns_info_t dns;
        dns.ip.u_addr.ip4.addr = addr;
        dns.ip.type = IPADDR_TYPE_V4;
        ESP_ERROR_CHECK(esp_netif_set_dns_info(netif, type, &dns));
    }
    return ESP_OK;
}

static void set_static_ip(esp_netif_t *netif)
{
    esp_err_t result = esp_netif_dhcpc_stop(netif);
    if ((result != ESP_OK) 
        &&  (result != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED)) {
        ESP_LOGE(TAG, "Failed to stop dhcp client: %d", result);
        return;
    }
    // Manually set ip
    esp_netif_ip_info_t ip;
    memset(&ip, 0 , sizeof(esp_netif_ip_info_t));
    ip.ip.addr = ipaddr_addr(FALLBACK_IP_ADDR);
    ip.netmask.addr = ipaddr_addr(FALLBACK_NETMASK_ADDR);
    ip.gw.addr = ipaddr_addr(FALLBACK_GW_ADDR);
    if (esp_netif_set_ip_info(netif, &ip) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set ip info");
        return;
    }
    // Set the IP obtained flag to true
    ip_obtained = true;
    ESP_LOGI(TAG, "Success to set static ip: %s, netmask: %s, gw: %s", FALLBACK_IP_ADDR, FALLBACK_NETMASK_ADDR, FALLBACK_GW_ADDR);
    // Set dns server
    ESP_ERROR_CHECK(set_dns_server(netif, ipaddr_addr(FALLBACK_MAIN_DNS_SERVER), ESP_NETIF_DNS_MAIN));
    ESP_ERROR_CHECK(set_dns_server(netif, ipaddr_addr(FALLBACK_BACKUP_DNS_SERVER), ESP_NETIF_DNS_BACKUP));
}

static void fallback_to_static_ip(esp_netif_t *netif, bool wait) {
    // Select and initialize basic parameters of the timer
    timer_config_t config = {
        .alarm_en = TIMER_ALARM_EN,
        .counter_en = TIMER_PAUSE,
        .counter_dir = TIMER_COUNT_UP,
        .divider = TIMER_DIVIDER,
    }; // default clock source is APB
    timer_init(TIMER_GROUP_0, TIMER_0, &config);
    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
    timer_start(TIMER_GROUP_0, TIMER_0);
    uint64_t task_counter_value;
    
    while (!ip_obtained) {
        timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &task_counter_value);
        if (!wait || (task_counter_value > DHCP_TIMEOUT)) {
            ESP_LOGI(TAG, "Setting static IP as fallback.");
            set_static_ip(netif);
        }
        else {
            ESP_LOGI(TAG, "(%llu) Waiting for IP...", task_counter_value);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void start_talkerlistener_task(void *pvParameters) {
    
    ESP_LOGI(TAG, "Starting TalkerListener...");

    esp_netif_t *eth_netif = (esp_netif_t *)pvParameters;
    //esp_netif_iodriver_handle handle = esp_netif_get_io_driver(eth_netif);

    Entity::CommonInformation commonInfo{
        /*entityID=*/UniqueIdentifier(ENTITY_ID),
        /*entityModelID=*/UniqueIdentifier(ENTITY_MODEL_ID),
        /*entityCapabilities=*/EntityCapabilities(EntityCapability::AemSupported),
        /*talkerStreamSources=*/1,
        /*talkerCapabilities=*/TalkerCapabilities(TalkerCapability::AudioSource),
        /*listenerStreamSinks=*/1,
        /*listenerCapabilities=*/ListenerCapabilities(ListenerCapability::AudioSink),
        /*controllerCapabilities=*/ControllerCapabilities(ControllerCapability::Implemented),
        /*identifyControlIndex=*/0,
        /*associationID=*/UniqueIdentifier(0x1122334455667788)
    };
    
    Entity::InterfaceInformation interfaceInfo{
        {0x00, 0x1A, 0xB6, 0x00, 0x02, 0x03}, 
        /*validTime=*/2, 
        /*availableIndex=*/0, 
        /*gptpGrandmasterID=*/UniqueIdentifier(0xFFEEDDCCBBAA9988), 
        /*gptpDomainNumber=*/0
    };
    
    // Create the Entity instance
    Entity entity{commonInfo, interfaceInfo};
    
    // Example EntityTree setup
    EntityTree entityModelTree;
    
    // Set up entityModelTree as required...
    // (Fill out entityModelTree's dynamicModel, staticModel, configurationTrees, etc.)
    
    // Instantiate the AtdeccTalkerListener
    AtdeccTalkerListener talkerListener{entity, &entityModelTree};
    
    // Open and configure L2 TAP File descriptor
    esp_err_t err = talkerListener.initialize(eth_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize AtdeccTalkerListener: %s", esp_err_to_name(err));
        goto error;
    } else {
        while (true) {
            // Send Entity Available message
            talkerListener.sendEntityAvailable(
                commonInfo.entityID.getValue(),
                commonInfo.entityModelID.getValue(),
                commonInfo.entityCapabilities.getValue()
            );
            ESP_LOGI(TAG, "Entity Available message sent.");

            // Send Entity Departing message (for example when shutting down)
            // talkerListener.sendEntityDeparting(
            //     commonInfo.entityID.getValue(),
            //     0  // Available index
            // );
            // ESP_LOGI(TAG, "Entity Departing message sent");

            // Send Entity Discover message
            // talkerListener.sendEntityDiscover(commonInfo.entityID.getValue());
            // ESP_LOGI(TAG, "Entity Discover message sent");
            
            // Send Connect Stream command (example of ACMP operation)
            // talkerListener.sendConnectStream();
            // ESP_LOGI(TAG, "Connect Stream command sent");
            
            vTaskDelay(pdMS_TO_TICKS(1000));
            //ESP_LOGI(TAG, "HEAP SIZE = %d", heap_caps_get_free_size(MALLOC_CAP_8BIT) );
        }
    }
error:
    vTaskDelete(NULL);
}

// Ethernet event handler
static void eth_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{	
    if (event_id == ETHERNET_EVENT_CONNECTED) {

        ESP_LOGI(TAG, "Ethernet Link Up");
        vTaskDelay(pdMS_TO_TICKS(500)); // Delay to ensure link is stable

        // Start DHCP
        esp_netif_t *eth_netif = (esp_netif_t *)arg;
        esp_netif_ip_info_t ip_info;
        static bool waitFirst = false; // flag to wait for dhcp to timeout or not
        //esp_netif_get_ip_info(eth_netif, &ip_info);
        if (esp_netif_get_ip_info(eth_netif, &ip_info) == ESP_OK) {
            if (ip_info.ip.addr != 0) {
                ESP_LOGI(TAG, "IP Address: " IPSTR, IP2STR(&ip_info.ip));
            } else {
                ESP_LOGI(TAG, "IP Address not assigned yet.");
                waitFirst = true; // wait for dhcp to time out before assigning static ip
                fallback_to_static_ip(eth_netif, waitFirst);
            }
        } else {
            ESP_LOGE(TAG, "Failed to get IP address, assigning manual address.");
            fallback_to_static_ip(eth_netif, waitFirst);
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
    
    esp_netif_t *eth_netif = (esp_netif_t *)arg;
    //xTaskCreate(eth_frame_logger, "frame_logger", 4096, NULL, 5, NULL);
    xTaskCreate(start_talkerlistener_task, "talker_listener", 4096, eth_netif, 5, NULL);
}

// Ethernet initialization function for ESP32 with LAN8720 PHY
static esp_netif_t* init_ethernet()
{
    // Initialize TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_init());

    // Create default event loop if not already created
    ESP_ERROR_CHECK(esp_event_loop_create_default());

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

    // Create netif for Ethernet
    esp_netif_config_t netif_config = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&netif_config);

    // Attach Ethernet driver to TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));

    // Register the event handler for Ethernet events, passing the netif as argument
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, eth_netif));

    // Register Got IP event handler
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, eth_netif));

    // Start the Ethernet driver
    ESP_ERROR_CHECK(esp_eth_start(eth_handle));

    return eth_netif;
}

extern "C" void app_main()
{
    
    
    // Initialize NVS (non-volatile storage)
    ESP_ERROR_CHECK(nvs_flash_init());

    // Wait a little for serial monitoring
    if (startup_wait) {
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    
    // Initialize Ethernet and establish a connection
    //static esp_netif_t *eth_netif = init_ethernet();
    init_ethernet();
}
