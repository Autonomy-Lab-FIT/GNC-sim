# GNC Simulator Installation Guide for Windows

## Prerequisites

Before starting the installation, ensure you have the following:
- **MATLAB** installed on your Windows system
- Administrative rights on your Windows machine

## Installation Steps

### Step 1: Clone the Repository

1. Choose a folder, preferably the Documents folder
2. Open command prompt normally (without admin rights)
3. Navigate into the chosen folder and clone the GitHub repository:
   ```
   cd Documents
   git clone https://github.com/Autonomy-Lab-FIT/GNC-sim.git
   ```

**Note:** If you don't have git installed, download it from the Git website first. When you run the clone command, it may ask for your login credentials (for private repositories).

### Step 2: Install Docker Desktop

1. Download Docker Desktop exe file from the official Docker website
   ```
   https://www.docker.com/products/docker-desktop/
   ```
2. Most of the windows based systems are based on AMD64 architecture, so from the above website download the exe file for AMD64. You can also verify your architecture in command prompt using the below command. If your architecture is ARM64 then download the exe file for ARM64 architecture
   ```
   echo %PROCESSOR_ARCHITECTURE%
   ```   
3. Run the exe file as administrator
4. Continue installation with the default settings

### Step 3: Install Ubuntu 24 from Microsoft Store

Since Docker is built on Linux, we need to emulate a Linux environment on Windows using WSL2:

1. Open Microsoft Store
2. Search and install **Ubuntu 24**
3. Restart the computer if needed

### Step 4: Configure Docker Desktop with WSL Integration

1. Open Docker Desktop application
2. You can skip the sign-in process if prompted
3. Click on the gear icon (Settings) located on the top right
4. Click on **Resources**
5. Select **WSL Integration**
6. Enable the button next to Ubuntu 24 (it should turn blue from gray)
7. Click **Apply & Restart** to apply the settings

### Step 5: Set Up Ubuntu Terminal

1. Go to the cmd (opened normally without admin rights)
2. On the top, you will find a dropdown button
3. Select **Ubuntu 24** from the dropdown list
4. Ubuntu 24 terminal will open
5. Set a username and password of your choice (ensure your username has no spaces)

### Step 6: Test Display Settings

1. In the Ubuntu terminal, install x11 apps (applications with UI):
   ```bash
   sudo apt install x11-apps
   ```

2. After installation, test the display forwarding:
   ```bash
   xclock
   ```

A clock UI should appear. This confirms that WSL is able to forward the UI between Ubuntu and Windows.

### Step 7: Pull Docker Image

Pull the Docker image from Docker Hub:
```bash
docker pull prenithreddy/gnc_simulator:latest
```

**Note:** This will take time to download the image from Docker Hub.

### Step 8: Navigate to Project Folder

Navigate to the folder where you cloned the GitHub repository:
```bash
cd /mnt/c/users/your_username/Documents/GNC-sim/
```

Replace `your_username` with your actual Windows username.

### Step 9: Create and Run Docker Container

Create a persistent container from the pulled image:
```bash
docker run -it \
  --name gnc_simulator \
  --privileged \
  --env="DISPLAY=:0" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$PWD:/root/sim_ws/GNC_algorithms" \
  prenithreddy/gnc_simulator:latest
```

### Step 10: Navigate to Simulator Folder

Check if you are in the following folder:
```bash
~/sim_ws/GNC_algorithms
```

If you aren't in this folder, navigate to it:
```bash
cd /root/sim_ws/GNC_algorithms
```

### Step 11: Start the Simulator

Run the following command to start the simulator:
```bash
./start_px4_gz_sim.sh
```

If the above command throws any errors, run these commands to fix line ending issues:
```bash
apt-get update && apt-get install dos2unix
dos2unix start_px4_gz_sim.sh
dos2unix kill_px4_processes.sh
```

Once the above step is finished, rerun the simulator start command:
```bash
./start_px4_gz_sim.sh
```

Select the drone model and the world file as per your choice.

### Step 12: Open MATLAB

1. In cmd, open another Ubuntu terminal from the dropdown menu
2. In the Ubuntu terminal, navigate to the MATLAB folder:
   ```bash
   cd /mnt/c/users/your_username/Documents/GNC-sim/MATLAB
   ```
   Replace `your_username` with your actual Windows username.

3. Open MATLAB from the Ubuntu terminal using the following command (this method will add all the MATLAB files to the folder path automatically):
   ```bash
   matlab
   ```

## Container Management Commands

After the initial setup, you can manage your persistent Docker container using these commands:

### Stop the Container
```bash
docker stop gnc_simulator
```

### Start the Container Again
```bash
docker start gnc_simulator
```

### Attach to the Running Container
```bash
docker attach gnc_simulator
```

### Execute Commands in the Container
```bash
docker exec -it gnc_simulator bash
```

### Check Container Status
```bash
docker ps -a
```

### Remove Container (if needed)
```bash
docker rm gnc_simulator
```

## Troubleshooting

- If you encounter display issues, ensure X11 forwarding is working by testing with `xclock`
- If Docker commands fail, make sure Docker Desktop is running and WSL integration is properly configured
- For line ending errors in shell scripts, use `dos2unix` command as shown in Step 11
- Ensure your Windows username path is correct when navigating to mounted directories

## Support

For issues related to the simulator or additional help, please refer to the project repository or contact the development team.
