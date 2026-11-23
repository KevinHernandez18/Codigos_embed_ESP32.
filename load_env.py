import json
from pathlib import Path


REQUIRED_KEYS = (
    "WIFI_SSID",
    "WIFI_PASSWORD",
    "MQTT_HOST",
    "MQTT_USER",
    "MQTT_PASSWORD",
    "MQTT_CA",
)


def read_env_file(path):
    lines = path.read_text(encoding="utf-8").splitlines()
    values = {}
    line_index = 0

    while line_index < len(lines):
        line = lines[line_index].strip()
        line_number = line_index + 1
        line_index += 1

        if not line or line.startswith("#"):
            continue
        if "=" not in line:
            raise ValueError(
                "{}:{}: expected KEY=value".format(path.name, line_number)
            )

        key, raw_value = line.split("=", 1)
        key = key.strip()
        raw_value = raw_value.strip()
        if not key or key in values:
            raise ValueError(
                "{}:{}: empty or duplicate key".format(path.name, line_number)
            )

        if raw_value.startswith(("'", '"')):
            quote = raw_value[0]
            parts = [raw_value[1:]]
            while not parts[-1].rstrip().endswith(quote):
                if line_index >= len(lines):
                    raise ValueError(
                        "{}:{}: unterminated quoted value".format(
                            path.name, line_number
                        )
                    )
                parts.append(lines[line_index])
                line_index += 1

            parts[-1] = parts[-1].rstrip()
            parts[-1] = parts[-1][:-1]
            value = "\n".join(parts)
        else:
            value = raw_value

        values[key] = value

    missing_keys = [key for key in REQUIRED_KEYS if not values.get(key, "").strip()]
    if missing_keys:
        raise ValueError(
            "{}: missing or empty keys: {}".format(
                path.name, ", ".join(missing_keys)
            )
        )

    ca = values["MQTT_CA"].strip()
    if not ca.startswith("-----BEGIN CERTIFICATE-----") or not ca.endswith(
        "-----END CERTIFICATE-----"
    ):
        raise ValueError(
            "{}: MQTT_CA must contain a complete PEM certificate".format(path.name)
        )
    values["MQTT_CA"] = ca
    return values


Import("env")

project_dir = Path(env.subst("$PROJECT_DIR"))
dotenv_path = project_dir / ".env"
secrets = read_env_file(dotenv_path)

build_dir = Path(env.subst("$BUILD_DIR"))
build_dir.mkdir(parents=True, exist_ok=True)

header_lines = ["#pragma once", ""]
for key in REQUIRED_KEYS:
    header_lines.append(
        "inline constexpr char {0}[] = {1};".format(
            key, json.dumps(secrets[key], ensure_ascii=False)
        )
    )

(build_dir / "secrets.h").write_text(
    "\n".join(header_lines) + "\n", encoding="utf-8"
)
env.Append(CPPPATH=[str(build_dir)])
