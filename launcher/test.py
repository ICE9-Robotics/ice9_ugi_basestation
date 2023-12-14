import sys
import ruamel.yaml

yaml_str_1 = """\
node_js: "0.10"
"""

yaml_str_2 = """\
before_script: "0.10"
"""

yaml = ruamel.yaml.YAML()
yaml.preserve_quotes = True
data = ruamel.yaml.load(yaml_str_1, preserve_quotes=True)
yaml.preserve_quotes = True
ruamel.yaml.safe_dump(data, sys.stdout)
print('=====')
data = ruamel.yaml.safe_load(yaml_str_2)
ruamel.yaml.safe_dump(data, sys.stdout)