#define DEBUG_BUILD



#include <string.h>

#include <drivers/device/i2c.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module.h>
#include <drivers/drv_hrt.h>

#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/battery_temperature.h>

using namespace time_literals;

#define LM75_BASE_ADDR 0x48

#define LM75_REG_ADDR_TEMP  0x00
#define LM75_REG_ADDR_CONF  0x01
#define LM75_REG_ADDR_THYST 0x02
#define LM75_REG_ADDR_TOS   0x03

class BattTempSensor : public device::I2C, public I2CSPIDriver<BattTempSensor>
{
public:
	BattTempSensor(const I2CSPIDriverConfig &config);
	virtual ~BattTempSensor() = default;

	static void print_usage();

	int init() override;
	int probe() override;

	void RunImpl();

private:
	static constexpr uint32_t BATT_TEMP_SENSOR_UPDATE_PERIOD_US = 1000000; //1 sec

	void start();

	int LM75_ReadTemperature(float &temperature);

	uORB::PublicationMulti<battery_temperature_s> _battery_temperature_pub{ORB_ID(battery_temperature)};
};

BattTempSensor::BattTempSensor(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config)
{
	// printf("Constructor\n");
}

int BattTempSensor::init()
{
	// printf("init\n");
	int ret = I2C::init();

	if (ret != OK) {
		return ret;
	}


	// kick off work queue
	ScheduleNow();

	return OK;
}


int BattTempSensor::probe()
{
	// printf("probe\n");
	return OK;
}


void BattTempSensor::print_usage()
{
	PRINT_MODULE_USAGE_NAME("batt_temp_sensor", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(LM75_BASE_ADDR);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}


extern "C" __EXPORT int batt_temp_sensor_main(int argc, char *argv[])
{
	using ThisDriver = BattTempSensor;

	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 100000;
	cli.i2c_address = LM75_BASE_ADDR;


	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}


	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_BAT_TEMP_SENSOR_DEVTYPE_LM75);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;

}



void BattTempSensor::RunImpl()
{
	// printf("Is running !\n");

	float temp_f = -77.0f;
	LM75_ReadTemperature(temp_f);

	uint64_t now = hrt_absolute_time();
	battery_temperature_s battery_temperature;
	//battery_temperature.temperature_deg = 36.6f + 0.1f * (now/1000000); //Test. Temperature rise 0.1C per second;
	battery_temperature.temperature_deg = temp_f;
	battery_temperature.timestamp = now;
	_battery_temperature_pub.publish(battery_temperature);

	ScheduleDelayed(BATT_TEMP_SENSOR_UPDATE_PERIOD_US);
}


int BattTempSensor::LM75_ReadTemperature(float &temperature)
{
	uint8_t data[2];
	data[0] = LM75_REG_ADDR_TEMP;
	int ret = transfer(data, 1, nullptr, 0);
	if(ret != OK)
	{
		// printf("Transfer 1 failed\n");
		return ret;
	}
	// printf("Transfer 1 OK\n");

	ret = transfer(nullptr, 0, data, 2);
		if(ret != OK)
	{
		// printf("Transfer 2 failed\n");
		return ret;
	}
	// printf("Transfer 2 OK\n");
	// printf("data[0]: %d\n", data[0]);
	// printf("data[1]: %d\n", data[1]);

	uint16_t t  = (data[0] << 8) | data[1];
	t >>= 5; //Only 11 high bits used

	int16_t fix_temp = *(int16_t *)(&t);
	float temp_f = (float)fix_temp /  8;

	temperature = temp_f;
	// printf("temperature: %4.2f\n", (double)temperature);

	return OK;
}
