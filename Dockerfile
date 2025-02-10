FROM ubuntu:22.04

ENV DEBIAN_FRONTEND="noninteractive"
ENV TZ=Europe/Paris

RUN apt-get -qq update && \
    apt-get -qq install -y wget unzip build-essential cmake ninja-build git python3 python3-pip unzip xxd libarchive-zip-perl \
    udev \
    libusb-1.0-0 \
    libncurses5 \
    gdb-multiarch

RUN mkdir tools

# Install gcc_arm_none_eabi_10_2020_q4_major
RUN wget -q https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu-rm/10-2020q4/gcc-arm-none-eabi-10-2020-q4-major-x86_64-linux.tar.bz2 && \
    tar -xjf gcc-arm-none-eabi-10-2020-q4-major-x86_64-linux.tar.bz2 -C /tools && \
    rm gcc-arm-none-eabi-10-2020-q4-major-x86_64-linux.tar.bz2
ENV PATH="/tools/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH"
ENV GNU_INSTALL_ROOT="/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/"

# Install JLinkExe
RUN wget --post-data "accept_license_agreement=accepted&non_emb_ctr=confirmed" "https://www.segger.com/downloads/jlink/JLink_Linux_V812b_x86_64.deb"
RUN /lib/systemd/systemd-udevd --daemon \
    && apt-get -y install ./JLink_Linux_V812b_x86_64.deb \
    && rm -rf /var/lib/apt/lists/*

# Install nRF command line tools for mergehex
RUN wget -q https://www.nordicsemi.com/-/media/Software-and-other-downloads/Desktop-software/nRF-command-line-tools/sw/Versions-10-x-x/10-13-0/nRF-Command-Line-Tools_10_13_0_Linux64.zip && \
    unzip nRF-Command-Line-Tools_10_13_0_Linux64.zip -d /tools && \
    rm nRF-Command-Line-Tools_10_13_0_Linux64.zip && \
    cd /tools/nRF-Command-Line-Tools_10_13_0_Linux64 && \
    tar -xf nRF-Command-Line-Tools_10_13_0_Linux-amd64.tar.gz && \
    rm nRF-Command-Line-Tools_10_13_0_Linux-amd64.tar.gz && \
    tar -xf nRF-Command-Line-Tools_10_13_0.tar && \
    rm nRF-Command-Line-Tools_10_13_0.tar
ENV PATH="/tools/nRF-Command-Line-Tools_10_13_0_Linux64/mergehex:$PATH"
ENV PATH="/tools/nRF-Command-Line-Tools_10_13_0_Linux64/nrfjprog:$PATH"

# Configure proper locale support
#RUN sed -i '/en_US.UTF-8/s/^# //g' /etc/locale.gen && locale-gen
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

# Install nrfutil
RUN pip3 install nrfutil
