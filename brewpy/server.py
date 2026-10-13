
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
from modbus_rtu_client import ModbusRTUClient,AsyncModbusRTUClient
import os

# --------------------------------------------------------
# Environment variables
# --------------------------------------------------------

MODBUS_RTU_PORT     = os.getenv("MODBUS_RTU_PORT")
MODBUS_RTU_BAUD     = os.getenv("MODBUS_RTU_BAUD", 38400)
MODBUS_RTU_PARITY   = os.getenv("MODBUS_RTU_PARITY", "N")
MODBUS_RTU_STOPBIT  = os.getenv("MODBUS_RTU_STOPBIT", 1)
MODBUS_RTU_BYTESIZE = os.getenv("MODBUS_RTU_BYTESIZE", 8)
MODBUS_RTU_TIMEOUT  = os.getenv("MODBUS_RTU_TIMEOUT", 0.3)


# --------------------------------------------------------
# DATASTORE BACKING (classic Modbus memory model)
# --------------------------------------------------------

def build_datastore():
    """
    Build the datastore the classic way with ModbusDeviceContext.
    Tables:
      di (discrete inputs), co (coils), ir (input regs), hr (holding regs)
    """
    # Coils: 
    coils = ModbusSequentialDataBlock(0, [0]*32)

    # Discrete inputs: 0
    discrete_inputs = ModbusSequentialDataBlock(0, [0])

    # Input registers: 
    input_registers = ModbusSequentialDataBlock(0, [0]*32)

    # Holding registers: 
    holding_registers = ModbusSequentialDataBlock(0, [0]*256)

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
    context = build_datastore()

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
            port=MODBUS_RTU_PORT,
            baudrate=MODBUS_RTU_BAUD,
            parity=MODBUS_RTU_PARITY,
            stopbits=MODBUS_RTU_STOPBIT,
            bytesize=MODBUS_RTU_BYTESIZE,
            timeout=MODBUS_RTU_TIMEOUT,
        )
    except:
        rtu_sync = None
    # Wrap sync RTU client with async wrapper
    async_rtu = AsyncModbusRTUClient(rtu_sync)
    await async_rtu.connect()
    log.info("RTU serial connected")


    plc = SoftPLC(context, async_rtu)
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