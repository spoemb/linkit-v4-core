FROM ubuntu:22.04

ENV DEBIAN_FRONTEND="noninteractive"
ENV TZ=Europe/Paris

# System packages
RUN apt-get -qq update && \
    apt-get -qq install -y \
    wget curl unzip build-essential cmake ninja-build git \
    python3 python3-pip \
    xxd libarchive-zip-perl \
    udev libusb-1.0-0 libncurses5 \
    gdb-multiarch && \
    rm -rf /var/lib/apt/lists/*

RUN mkdir -p /tools

# ARM GCC Toolchain 10.3-2021.10 (recommended for nRF5 SDK 17)
RUN wget -q https://developer.arm.com/-/media/Files/downloads/gnu-rm/10.3-2021.10/gcc-arm-none-eabi-10.3-2021.10-x86_64-linux.tar.bz2 && \
    tar -xjf gcc-arm-none-eabi-10.3-2021.10-x86_64-linux.tar.bz2 -C /tools && \
    rm gcc-arm-none-eabi-10.3-2021.10-x86_64-linux.tar.bz2
ENV PATH="/tools/gcc-arm-none-eabi-10.3-2021.10/bin:$PATH"
ENV GNU_INSTALL_ROOT="/tools/gcc-arm-none-eabi-10.3-2021.10/bin/"

# nRF Command Line Tools (nrfjprog + mergehex)
RUN wget -q https://nsscprodmedia.blob.core.windows.net/prod/software-and-other-downloads/desktop-software/nrf-command-line-tools/sw/versions-10-x-x/10-24-2/nrf-command-line-tools-10.24.2_linux-amd64.tar.gz && \
    mkdir -p /tools/nrf-command-line-tools && \
    tar -xzf nrf-command-line-tools-10.24.2_linux-amd64.tar.gz -C /tools/nrf-command-line-tools --strip-components=1 && \
    rm nrf-command-line-tools-10.24.2_linux-amd64.tar.gz && \
    # Install bundled J-Link if present \
    JLINK_DEB=$(find /tools/nrf-command-line-tools -name "JLink_Linux_*.deb" 2>/dev/null | head -1) && \
    if [ -n "$JLINK_DEB" ]; then \
        /lib/systemd/systemd-udevd --daemon 2>/dev/null || true; \
        dpkg -i "$JLINK_DEB" 2>/dev/null || true; \
    fi
ENV PATH="/tools/nrf-command-line-tools/bin:$PATH"

# nrfutil CLI + nrf5sdk-tools plugin (for DFU package generation)
RUN curl -fsSL "https://developer.nordicsemi.com/.pc-tools/nrfutil/x64-linux/nrfutil" -o /usr/local/bin/nrfutil && \
    chmod +x /usr/local/bin/nrfutil && \
    nrfutil install nrf5sdk-tools

# Locale
ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US:en
ENV LC_ALL=en_US.UTF-8

WORKDIR /workspace
