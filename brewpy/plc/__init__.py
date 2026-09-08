"""
Re-export SoftPLC so existing call sites can keep doing:

    from plc import SoftPLC

without needing to know it now lives in plc/softplc.py.
"""
from plc.softplc import SoftPLC

__all__ = ["SoftPLC"]
