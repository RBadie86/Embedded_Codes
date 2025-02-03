# Use Jenkins base image with OpenJDK 21.0.6
FROM jenkins/jenkins:2.414.2-jdk21

# Switch to root for installing dependencies
USER root

# Update package list and install necessary tools
RUN apt-get update && apt-get install -y \
    lsb-release \
    python3.12 \
    python3.12-venv \
    python3.12-dev \
    python3-pip \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Set Python 3.12 as the default
RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.12 1

# Install Docker CLI
RUN curl -fsSLo /usr/share/keyrings/docker-archive-keyring.asc \
    https://download.docker.com/linux/debian/gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.asc] \
    https://download.docker.com/linux/debian $(lsb_release -cs) stable" > /etc/apt/sources.list.d/docker.list && \
    apt-get update && apt-get install -y docker-ce-cli

# Switch back to Jenkins user
USER jenkins

# Ensure Jenkins Plugin CLI works
RUN jenkins-plugin-cli --plugins "docker-workflow:1.28" || echo "Plugin installation failed, will retry in container startup"

# Expose Jenkins default port
EXPOSE 8080

# Set default command
CMD ["/usr/bin/tini", "--", "/usr/local/bin/jenkins.sh"]



