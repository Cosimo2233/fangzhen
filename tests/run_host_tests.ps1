$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $PSScriptRoot
$output = Join-Path $env:TEMP "artemis-firmware-host-tests.exe"
$sources = @(
    (Join-Path $root "artemis_protocol.c"),
    (Join-Path $root "artemis_controller.c"),
    (Join-Path $root "artemis_runtime_params.c"),
    (Join-Path $root "line_indicator.c"),
    (Join-Path $PSScriptRoot "test_artemis.c")
)

& gcc -std=c11 -Wall -Wextra -Werror -pedantic "-I$root" @sources -o $output
if ($LASTEXITCODE -ne 0) {
    throw "Host test compilation failed with exit code $LASTEXITCODE"
}

& $output
if ($LASTEXITCODE -ne 0) {
    throw "Host tests failed with exit code $LASTEXITCODE"
}
