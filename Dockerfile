# Use the official OpenJDK 21 image as the base
FROM eclipse-temurin:21.0.6_7-jdk

# Install Python 3.12 and necessary build tools
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    python3.12 \
    python3.12-venv \
    python3.12-dev \
    python3-pip \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Set Python 3.12 as the default python3
RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.11 1

# Create a user for Jenkins
RUN useradd -m -d /home/jenkins -s /bin/bash jenkins

# Set environment variables for Jenkins agent
ENV JENKINS_AGENT_HOME=/home/jenkins/agent
RUN mkdir -p $JENKINS_AGENT_HOME && chown -R jenkins:jenkins $JENKINS_AGENT_HOME

# Switch to the Jenkins user
USER jenkins
WORKDIR /home/jenkins

# Download the Jenkins agent JAR file
RUN curl --create-dirs -sSLo agent.jar https://repo.jenkins-ci.org/public/org/jenkins-ci/main/remoting/3107.v665000b_51092/remoting-3107.v665000b_51092.jar

# Set the entrypoint to run the Jenkins agent
ENTRYPOINT ["java", "-jar", "agent.jar"]


