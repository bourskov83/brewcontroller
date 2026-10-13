"""
Strict YAML loading for the config files.

PyYAML's safe_load silently keeps the LAST value when a key appears twice,
so a copy/pasted tag in tags.yaml would quietly replace the first one.
load_yaml() raises instead. Overriding a key that came in through a merge
(`<<: *temp` followed by `scale: 50`) is still allowed, because that is
the whole point of merge keys.
"""
from pathlib import Path

import yaml

_MERGE_TAG = "tag:yaml.org,2002:merge"


class _StrictLoader(yaml.SafeLoader):
    def construct_mapping(self, node, deep=False):
        if isinstance(node, yaml.MappingNode):
            seen = set()
            for key_node, _ in node.value:
                if key_node.tag == _MERGE_TAG or not isinstance(key_node, yaml.ScalarNode):
                    continue
                key = self.construct_object(key_node, deep=True)
                if key in seen:
                    raise yaml.constructor.ConstructorError(
                        None, None, f"duplicate key {key!r}", key_node.start_mark
                    )
                seen.add(key)
        return super().construct_mapping(node, deep=deep)


def load_yaml(path: str | Path):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.load(f, Loader=_StrictLoader)
