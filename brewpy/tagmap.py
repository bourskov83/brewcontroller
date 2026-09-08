from logger_config import log
from dataclasses import dataclass
from typing import Dict, Any
import yaml

#CO, DI, IR, HR = 1, 2, 3, 4

@dataclass(frozen=True)
class Tag:
    table: int
    addr: int

class Tags:
    def __init__(self, store, mapping: Dict[str, Tag]):
        self.store = store
        self.map = mapping

    def read(self, name: str, count: int = 1) -> list:
        t = self.map[name]
        return self.store.getValues(t.table, t.addr, count=count)

    def write(self, name: str, values):
        t = self.map[name]
        vals = values if isinstance(values, list) else [values]
        vals = [v & 0xFFFF if isinstance(v, int) else v for v in vals] # handle negative registers!
        self.store.setValues(t.table, t.addr, vals)



def load_tags(path: str = "tags.yaml") -> Dict[str, "Tag"]:
    with open(path, "r", encoding="utf-8") as f:
        doc: Dict[str, Any] = yaml.safe_load(f) or {}

    out: Dict[str, Tag] = {}

    for name, spec in doc.items():

        table_code = int(spec["table"])
        addr = int(spec["address"])
        out[name] = Tag(table_code, addr)

    return out
