"""
Moondream + Ollama setup for Kaggle, written as a pure Python script.

Run:
    python llava.py

Environment:
    NGROK_AUTHTOKEN=<your-ngrok-token>   # optional, required only for public tunnel

The script:
1. Installs zstd when apt is available.
2. Installs Ollama if it is not already installed.
3. Checks the NVIDIA GPU.
4. Starts the Ollama server.
5. Pulls the Moondream model.
6. Verifies Ollama and Moondream.
7. Optionally creates an ngrok tunnel when NGROK_AUTHTOKEN is supplied.
"""

import os
import shutil
import subprocess
import sys
import time
from pathlib import Path
from urllib.request import urlopen
from urllib.error import URLError

# =====================================================================
# AUTOMATIC DEPENDENCY INSTALLER (Runs directly inside the Kaggle cloud)
# =====================================================================
def install_dependencies():
    """Dynamically installs any required Python libraries on the fly."""
    required_packages = ["pyngrok"]  # Add any other missing pip packages here
    
    for package in required_packages:
        try:
            __import__(package)
        except ImportError:
            print(f"📦 Library '{package}' not found. Installing on Kaggle cloud...")
            subprocess.check_call([sys.executable, "-m", "pip", "install", "--quiet", package])
            print(f"✓ '{package}' installed successfully.")

# Run the installer immediately upon execution
install_dependencies()


OLLAMA_HOST = os.environ.get("OLLAMA_HOST", "127.0.0.1:11434")
OLLAMA_URL = f"http://{OLLAMA_HOST}"
MODEL_NAME = "moondream"
WORK_DIR = Path("/kaggle/working") if Path("/kaggle/working").exists() else Path.cwd()
NGROK_DIR = WORK_DIR / "ngrok_bin"
OLLAMA_LOG = WORK_DIR / "ollama_daemon.log"
NGROK_AUTHTOKEN = "34GP1f4njzBBTVHJNNhVcXrsuLn_7WBATL6TtWZFhhCWnCuut"

daemon_process = None


def run(command, *, check=True, capture_output=False, env=None, cwd=None):
    """Run a command without relying on notebook shell syntax."""
    print(f"\n$ {' '.join(map(str, command))}")
    return subprocess.run(
        [str(x) for x in command],
        check=check,
        capture_output=capture_output,
        text=True,
        env=env,
        cwd=cwd,
    )


def install_zstd():
    """Install zstd on Debian/Ubuntu/Kaggle when apt-get is available."""
    apt = shutil.which("apt-get")
    if not apt:
        print("apt-get not available; skipping zstd installation.")
        return

    print("\n[1/7] Installing/checking zstd...")
    try:
        run([apt, "update"], check=True)
        run([apt, "install", "-y", "zstd"], check=True)
        print("✓ zstd is ready.")
    except subprocess.CalledProcessError as exc:
        print(f"⚠ zstd installation failed (continuing): {exc}")


def install_ollama():
    """Install Ollama if it is missing."""
    print("\n[2/7] Checking Ollama installation...")
    ollama = shutil.which("ollama")

    if ollama:
        print(f"✓ Ollama is already installed: {ollama}")
    else:
        print("Ollama not found. Installing...")

        curl = shutil.which("curl")
        if not curl:
            raise RuntimeError("curl is required to install Ollama.")

        install_cmd = f"{curl} -fsSL https://ollama.com/install.sh | sh"
        result = subprocess.run(
            ["bash", "-c", install_cmd],
            capture_output=True,
            text=True,
        )

        if result.returncode != 0:
            print(result.stdout)
            print(result.stderr)
            raise RuntimeError("Ollama installation failed.")

        ollama = shutil.which("ollama")
        if not ollama:
            for candidate in ("/usr/local/bin/ollama", "/usr/bin/ollama"):
                if Path(candidate).exists():
                    ollama = candidate
                    break

        if not ollama:
            raise RuntimeError(
                "Ollama installation completed, but the executable was not found."
            )

        print(f"✓ Ollama installed: {ollama}")

    result = run(
        [ollama, "--version"],
        capture_output=True,
        check=True,
    )
    print("Ollama version:", result.stdout.strip())
    return ollama


def check_gpu():
    """Display NVIDIA GPU information if nvidia-smi is available."""
    print("\n[3/7] Checking NVIDIA GPU...")
    nvidia_smi = shutil.which("nvidia-smi")

    if not nvidia_smi:
        print("⚠ nvidia-smi was not found.")
        return False

    result = run([nvidia_smi], capture_output=True, check=False)

    if result.returncode == 0:
        print("✓ NVIDIA GPU detected.\n")
        print(result.stdout)
        return True

    print("⚠ NVIDIA GPU could not be queried.")
    print(result.stderr)
    return False


def ollama_is_running():
    """Check the Ollama HTTP API."""
    try:
        with urlopen(f"{OLLAMA_URL}/api/tags", timeout=5) as response:
            return response.status == 200
    except (URLError, TimeoutError, OSError):
        return False


def start_ollama(ollama):
    """Start Ollama's server if it is not already running."""
    global daemon_process

    print("\n[4/7] Starting Ollama server...")

    if ollama_is_running():
        print("✓ Ollama server is already running.")
        return

    env = os.environ.copy()
    env["OLLAMA_HOST"] = OLLAMA_HOST

    log_handle = open(OLLAMA_LOG, "a", buffering=1)

    daemon_process = subprocess.Popen(
        [ollama, "serve"],
        stdout=log_handle,
        stderr=subprocess.STDOUT,
        env=env,
    )

    print(f"✓ Ollama process started. PID: {daemon_process.pid}")
    print("Waiting for Ollama server", end="", flush=True)

    for _ in range(30):
        time.sleep(1)
        print(".", end="", flush=True)

        if ollama_is_running():
            print("\n✓ Ollama server is ready.")
            return

    print("\n❌ Ollama server failed to start.")
    print(f"\n--- Ollama log: {OLLAMA_LOG} ---")
    if OLLAMA_LOG.exists():
        print(OLLAMA_LOG.read_text(errors="replace"))

    raise RuntimeError("Ollama server did not start.")


def pull_moondream(ollama):
    """Download the Moondream model through Ollama."""
    print(f"\n[5/7] Downloading {MODEL_NAME}...")
    env = os.environ.copy()
    env["OLLAMA_HOST"] = OLLAMA_HOST

    result = run(
        [ollama, "pull", MODEL_NAME],
        env=env,
        check=False,
    )

    if result.returncode != 0:
        raise RuntimeError(f"Failed to download {MODEL_NAME}.")

    print(f"✓ {MODEL_NAME} downloaded successfully.")


def verify(ollama):
    """Verify the server and model."""
    print("\n[6/7] Verifying Ollama + Moondream...")

    if not ollama_is_running():
        raise RuntimeError(f"Cannot connect to Ollama at {OLLAMA_URL}")

    print("✓ Ollama server is running.")

    result = run(
        [ollama, "list"],
        capture_output=True,
        check=True,
    )

    print("\nInstalled models:")
    print(result.stdout)

    if MODEL_NAME.lower() not in result.stdout.lower():
        raise RuntimeError(f"{MODEL_NAME} was not found in Ollama.")

    print(f"✓ {MODEL_NAME} is installed.")


def start_ngrok():
    """Optionally expose Ollama through ngrok."""
    print("\n[7/7] Configuring ngrok...")

    token = NGROK_AUTHTOKEN
    if not token:
        print("⚠ NGROK_AUTHTOKEN is not set.")
        return None

    try:
        from pyngrok import ngrok, conf
    except ImportError:
        print("pyngrok is not installed.")
        return None

    NGROK_DIR.mkdir(parents=True, exist_ok=True)

    config = conf.get_default()
    config.config_path = str(NGROK_DIR / "ngrok.yml")
    config.ngrok_path = str(NGROK_DIR / "ngrok")

    print("Applying ngrok authentication token...")
    ngrok.set_auth_token(token)

    try:
        ngrok.kill()

        tunnel = ngrok.connect(
            11434,
            proto="http",
            host_header="localhost:11434",
        )

        print("\n" + "=" * 60)
        print("🔗 NGROK TUNNEL READY")
        print(f"Public endpoint: {tunnel.public_url}")
        print("=" * 60)

        return tunnel
    except Exception as exc:
        print(f"⚠ ngrok initialization failed: {exc}")
        return None


def main():
    print("=" * 60)
    print("🚀 STARTING MOONDREAM + OLLAMA CLOUD INFRASTRUCTURE")
    print("=" * 60)

    try:
        install_zstd()
        ollama = install_ollama()
        check_gpu()
        start_ollama(ollama)
        pull_moondream(ollama)
        verify(ollama)
        
        tunnel = start_ngrok()
        if tunnel:
            print(f"\n🎯 [SUCCESS] Public Endpoint established at: {tunnel.public_url}")
        else:
            print("\n⚠️ [WARNING] Ngrok tunnel could not be created. Check your token!")

    except Exception as e:
        print(f"\n❌ [CRITICAL ERROR DURING SETUP]: {e}")
        print("Forcing keep-alive loop anyway so you can debug the instance logs...")

    keep_alive()

def monitor_kill_signal():
    """Polls Ngrok's local API to see if the host requested a shutdown."""
    import requests
    import time
    import os
    print("[*] Monitoring Ngrok logs for kill signal (/api/kill_dhruv)...")
    while True:
        time.sleep(5)
        try:
            resp = requests.get("http://localhost:4040/api/requests/http?limit=10", timeout=2)
            if resp.status_code == 200:
                data = resp.json()
                for req in data.get("requests", []):
                    if "/api/kill_dhruv" in req.get("request", {}).get("uri", ""):
                        print("\n[💀] Kill signal detected in Ngrok logs! Shutting down Kaggle kernel gracefully.")
                        os.system("pkill -f ollama")
                        os._exit(0)
        except Exception:
            pass

def keep_alive():
    """Simple loop to keep the Kaggle script running."""
    import threading
    import time
    threading.Thread(target=monitor_kill_signal, daemon=True).start()
    
    print("\n🔒 Entering persistent runtime loop. Keeping server alive...")
    counter = 0
    while True:
        counter += 1
        print(f"⏱️ [SERVER ALIVE] Running continuously for {counter * 30} seconds... Active GPU listening.", flush=True)
        time.sleep(30)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
        sys.exit(130)
    except Exception as exc:
        print(f"\n❌ ERROR: {exc}")
        sys.exit(1)