FROM jenkins/jenkins:lts
USER root  # Switch to root to install packages

# Install Python 3.12
RUN apt update && apt install -y \
    software-properties-common \
    && add-apt-repository ppa:deadsnakes/ppa -y \
    && apt update && apt install -y python3.12 python3.12-venv python3.12-dev \
    && rm -rf /var/lib/apt/lists/*

USER jenkins  # Switch back to Jenkins user
