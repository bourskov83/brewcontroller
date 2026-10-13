
import asyncio
from logger_config import log
from pymodbus import FramerType
from pymodbus.server import StartAsyncTcpServer
from pymodbus.datastore import (
    ModbusSequentialDataBlock,
    ModbusDeviceContext,    
    ModbusServerContext,
)
from pymodbus import ModbusDeviceIdentification
from plc import SoftPLC
from tagmap import table_sizes
from check_config import load_all, validate_or_raise
from modbus_rtu_client import ModbusRTUClient,AsyncModbusRTUClient


# --------------------------------------------------------
# DATASTORE BACKING (classic Modbus memory model)
# --------------------------------------------------------

def build_datastore(tag_defs):
    """
    Build the datastore from tags.yaml: every table is exactly as large as its
    highest tag address needs (minimum 32 entries).

    Blocks START AT 1: pymodbus adds 1 to every address internally, so a block
    that starts at 0 is one entry short and the highest tag would silently fail.
    """
    n = table_sizes(tag_defs, minimum=32)

    coils = ModbusSequentialDataBlock(1, [False] * n["coil"])
    discrete_inputs = ModbusSequentialDataBlock(1, [0] * n["di"])
    input_registers = ModbusSequentialDataBlock(1, [0] * n["ir"])
    holding_registers = ModbusSequentialDataBlock(1, [0] * n["hr"])

    # NOTE: In 3.10+ use ModbusDeviceContext (not ModbusSlaveContext)
    device_ctx = ModbusDeviceContext(
        di=discrete_inputs,
        co=coils,
        ir=input_registers,
        hr=holding_registers,
    )

    return ModbusServerContext(devices=device_ctx, single=True)

# --------------------------------------------------------
# MAIN SERVER SETUP
# --------------------------------------------------------

async def main():
    # tags.yaml + esp32.yaml + sensors.yaml: load once, cross-check, then share
    tag_defs, device, sensor_map = load_all()
    validate_or_raise(tag_defs, device, sensor_map, log)   # a bad config stops the boot here
    context = build_datastore(tag_defs)

    identity = ModbusDeviceIdentification(
        info_name={
            "VendorName": "SoftPLC",
            "ProductName": "SoftPLC-Datastore",
            "ModelName": "v1",
            "MajorMinorRevision": "1.0",
        }
    )
    try:
        rtu_sync = ModbusRTUClient(
            port=device.link.port,
            baudrate=device.link.baud,
            parity="N",
            stopbits=1,
            bytesize=8,
            timeout=device.link.timeout,
        )
    except:
        rtu_sync = None
    # Wrap sync RTU client with async wrapper
    async_rtu = AsyncModbusRTUClient(rtu_sync)
    await async_rtu.connect()
    log.info("RTU serial connected")


    plc = SoftPLC(context, async_rtu, tag_defs, device, sensor_map)
    await plc.start()


    # Start background tasks
    #scan_task = asyncio.create_task(plc.scan_task())
    #bridge_task = asyncio.create_task(plc.modbus_bridge_task())

    try: 
        # Start Modbus TCP server
        await StartAsyncTcpServer(
            context=context,
            identity=identity,
            address=("0.0.0.0", 5020),
            framer=FramerType.SOCKET,
        )
    finally:
        # Graceful shutdown
     
        await plc.stop()
        await async_rtu.close()
        log.info("RTU serial closed")
   

if __name__ == "__main__":
    asyncio.run(main())