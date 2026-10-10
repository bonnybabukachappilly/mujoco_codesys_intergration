from yaml import SafeDumper


class NoAliasDumper(SafeDumper):
    def ignore_aliases(self, data):
        return True
