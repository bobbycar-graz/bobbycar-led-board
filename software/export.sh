FIRMWARE_ROOT="$(dirname "$BASH_SOURCE")"

if [[ ! -f "${FIRMWARE_ROOT}/esp-idf/export.sh" ]]
then
    echo "esp-idf is missing, please check out all needed submodules!"
    echo "git submodule update --init --recursive"
    return
fi

if [[ ! $PATH =~ $(pwd)/tools ]]
then
    export PATH="$(pwd)/tools:$PATH"
fi

. ${FIRMWARE_ROOT}/esp-idf/export.sh
