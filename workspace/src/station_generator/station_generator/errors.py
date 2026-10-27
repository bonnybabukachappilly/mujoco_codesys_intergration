class GeneratorError(Exception):
    exit_code = 1


class ConfigError(GeneratorError):
    exit_code = 2


class EmitError(GeneratorError):
    exit_code = 3
