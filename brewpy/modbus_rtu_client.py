

from logger_config import log
from pymodbus.client import ModbusSerialClient
from pymodbus import FramerType
import time
import asyncio
from typing import Optional, Sequence


class ModbusRTUError(Exception):
    """Generic Modbus I/O error."""

class ModbusRTUHREGReadError(ModbusRTUError):
    def __init__(self, address: int, count: int, original: Exception | None = None, detail: str | None = None):
        msg = f"Failed to read HREGs at addr={address}, count={count}"
        if detail:
            msg += f" ({detail})"
        super().__init__(msg)
        self.address = address
        self.count = count
        self.original = original

class ModbusRTUHREGWriteError(ModbusRTUError):
    def __init__(self, address: int, values: list[int], original: Exception | None = None, detail: str | None = None):
        msg = f"Failed to write HREGs at addr={address}, values={values}"
        if detail:
            msg += f" ({detail})"
        super().__init__(msg)
        self.address = address
        self.values = values
        self.original = original

class ModbusRTUCoilsWriteError(ModbusRTUError):
    def __init__(self, address: int, values: list[int], original: Exception | None = None, detail: str | None = None):
        msg = f"Failed to write Coils at addr={address}, values={values}"
        if detail:
            msg += f" ({detail})"
        super().__init__(msg)
        self.address = address
        self.values = values
        self.original = original


class ModbusRTUClient:
    def __init__(
            self, 
            port: str,
            baudrate: int = 115200,
            parity: str = "N",
            stopbits: int = 1,
            bytesize: int = 8,
            timeout: float = 2.0,
    ):
        self.port=port
        self.baudrate=baudrate
        self.parity=parity
        self.stopbits=stopbits
        self.bytesize=bytesize
        self.timeout=timeout
        self.session = ModbusSerialClient(
            port=self.port,
            baudrate=self.baudrate,
            parity=self.parity,
            stopbits=self.stopbits,
            bytesize=self.bytesize,
            timeout=self.timeout,
            framer=FramerType.RTU

        )
    def connect(self):
        log.debug("RTU connect()")
        if not self.session.connect():
            raise ModbusRTUError(f"Could not open serial port: {self.port}")
        
    def close(self):
        self.session.close()

    def read_hregs(self, address: int, count: int = 1) -> list[int]:
        try:
            rr = self.session.read_holding_registers(address=address, count=count)
            if rr.isError():
                # If the library exposes an error string, include it
                detail = getattr(rr, "message", None) or rr.__class__.__name__
                raise ModbusRTUHREGReadError(address, count, detail=detail)

            return rr.registers

        except Exception as e:
            log.error(f"Cannot read HREG:[{address}], count:{count}!")
            log.debug(e)
            raise ModbusRTUHREGReadError(address, count, original=e)
        

    def write_hregs(self, address: int, values: list[int]) -> bool:
        try:
            wr = self.session.write_registers(address=address, values=values)
            if wr.isError():
                # If the library exposes an error string, include it
                detail = getattr(wr, "message", None) or wr.__class__.__name__
                raise ModbusRTUHREGWriteError(address, values, detail=detail)

            return True

        except Exception as e:
            log.error(f"Cannot write HREG:[{address}], values:{values}!")
            log.debug(e)
            raise ModbusRTUHREGWriteError(address, values, original=e)
        
    def write_coils(self, address: int, values: list[bool]) -> bool:
        try:
            wc = self.session.write_coils(address=address, values=values)
            if wc.isError():
                detail = getattr(wc, "message", None) or wc.__class__.__name__
                raise ModbusRTUCoilsWriteError(address, values, detail=detail)
            return True
        except Exception as e:
            log.error(f"Cannot write coils:[{address}], values:{values}!")
            log.debug(e)
            raise ModbusRTUCoilsWriteError(address, values, original=e)


class AsyncModbusRTUClient:
    """
    Async adaptor for exposing non-blocking ModbusRTUClient.
    """

    def __init__(self, base: ModbusRTUClient, lock: Optional[asyncio.Lock] = None):
        self._base = base
        self._lock = lock or asyncio.Lock()

    async def connect(self) -> None:
        await asyncio.to_thread(self._base.connect)

    async def close(self) -> None:
        await asyncio.to_thread(self._base.close)

    async def __aenter__(self):
        await self.connect()
        return self

    async def __aexit__(self, exc_type, exc, tb):
        await self.close()


    async def write_hregs(
        self, addr: int, values: Sequence[int],
        *, timeout: float | None = 2.0, retries: int = 1, backoff_base: float = 0.05
    ) -> None:
        attempt = 0
        last_exc: Exception | None = None
        while attempt <= retries:
            try:
                async with self._lock:
                    coro = asyncio.to_thread(self._base.write_hregs, addr, list(values))
                    ok = await (asyncio.wait_for(coro, timeout) if timeout else coro)
                if not bool(ok):
                    # The sync client would already have raised on error,
                    # but keep a defensive check:
                    raise ModbusRTUHREGWriteError(addr, list(values), detail="driver reported False")
                log.debug(f"RTU write_hregs addr={addr} values={values} ok={ok}")
                return
            except asyncio.CancelledError:
                raise
            except Exception as e:
                last_exc = e
                if attempt == retries:
                    break
                await asyncio.sleep(backoff_base * (2 ** attempt))
                attempt += 1
        raise ModbusRTUHREGWriteError(addr, list(values), original=last_exc)

    async def try_write_hregs(self, addr: int, values: Sequence[int], **kw) -> bool:
        try:
            await self.write_hregs(addr, values, **kw)
            return True
        except Exception as e:
            log.debug("try_write_hregs suppressed: %s", e)
            return False

    async def write_coils(
        self, addr: int, values: Sequence[bool],
        *, timeout: float | None = 2.0, retries: int = 1, backoff_base: float = 0.05
    ) -> None:
        
        attempt = 0
        last_exc: Exception | None = None
        
        while attempt <= retries:
            try:
                async with self._lock:
                    coro = asyncio.to_thread(self._base.write_coils, addr, list(values))
                    ok = await (asyncio.wait_for(coro, timeout) if timeout else coro)
                if not bool(ok):
                    raise ModbusRTUCoilsWriteError(addr, list(values), detail="driver reported False")
                log.debug(f"RTU write_coils addr={addr} values={values} ok={ok}")
                return
            except asyncio.CancelledError:
                raise
            except Exception as e:
                last_exc = e
                if attempt == retries:
                    break
                await asyncio.sleep(backoff_base * (2 ** attempt))
                attempt += 1
        raise ModbusRTUCoilsWriteError(addr, list(values), original=last_exc)

    async def try_write_coils(self, addr: int, values: Sequence[bool], **kw) -> bool:
        try:
            await self.write_coils(addr, values, **kw)
            return True
        except Exception as e:
            log.debug("try_write_coils suppressed: %s", e)
            return False

    async def read_hregs(
        self, addr: int, count: int,
        *, timeout: float | None = 2.0, retries: int = 1, backoff_base: float = 0.05
    ) -> list[int]:
        attempt = 0
        last_exc: Exception | None = None
        while attempt <= retries:
            try:
                async with self._lock:
                    coro = asyncio.to_thread(self._base.read_hregs, addr, count)
                    result = await (asyncio.wait_for(coro, timeout) if timeout else coro)

                if not isinstance(result, (list, tuple)):
                    raise ModbusRTUHREGReadError(addr, count, detail=f"unexpected result type {type(result)}")
                values = list(result)
                if len(values) != count:
                    raise ModbusRTUHREGReadError(addr, count, detail=f"length mismatch: expected {count}, got {len(values)}")

  #              log.debug("RTU read_hregs addr=%s count=%s values=%s", addr, count, values)
                return values
            
            except asyncio.CancelledError:
                raise
            except Exception as e:
                last_exc = e
                if attempt == retries:
                    break
                await asyncio.sleep(backoff_base * (2 ** attempt))
                attempt += 1
        raise ModbusRTUHREGReadError(addr, count, original=last_exc)

    async def read_hregs_or_none(self, addr: int, count: int, **kw) -> list[int] | None:
        """Forgiving variant: returns None instead of raising on I/O failure."""
        try:
            return await self.read_hregs(addr, count, **kw)
        except Exception as e:
            log.debug("read_hregs_or_none suppressed: %s", e)
            return None





