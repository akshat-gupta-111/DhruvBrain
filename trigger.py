"""
Kaggle Moondream Trigger Automation & URL Resolver
===================================================
1. Automatically packages llava.py into automated_run.ipynb
2. Pushes kernel to Kaggle GPU (if not already running)
3. Streams execution logs to automatically capture the public Ngrok tunnel URL
4. Returns the active URL and persists it to .env
"""

import os
import re
import sys
import json
import time
import subprocess
from pathlib import Path
from typing import Optional

# Ensure UTF-8 printing safely on Windows consoles
if hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

BASE_DIR = Path(__file__).parent.resolve()
KERNEL_ID = "akshatgupta2006/local-automation-test"
SCRIPT_FILE = BASE_DIR / "llava.py"
NOTEBOOK_FILE = BASE_DIR / "automated_run.ipynb"
ENV_FILE = BASE_DIR / ".env"

# Ensure Kaggle credentials are discoverable by Kaggle CLI/API
KAGGLE_JSON = BASE_DIR / "kaggle.json"
if KAGGLE_JSON.exists():
    os.environ["KAGGLE_CONFIG_DIR"] = str(BASE_DIR)
else:
    # Under systemd boot, HOME might not be set. Try to fallback to explicitly defining the default path
    if "HOME" not in os.environ:
        os.environ["KAGGLE_CONFIG_DIR"] = "/home/dhruv/.kaggle"

# Alternatively, parse credentials directly from .env (most robust for systemd)
if ENV_FILE.exists():
    with open(ENV_FILE, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line.startswith("KAGGLE_USERNAME="):
                os.environ["KAGGLE_USERNAME"] = line.split("=", 1)[1].strip()
            elif line.startswith("KAGGLE_KEY="):
                os.environ["KAGGLE_KEY"] = line.split("=", 1)[1].strip()


def get_kaggle_cmd() -> str:
    """Finds the kaggle executable, checking PATH and common virtual environments."""
    venv_kaggle = BASE_DIR.parent / "Trigger" / ".venv" / "Scripts" / "kaggle.exe"
    if venv_kaggle.exists():
        return str(venv_kaggle)

    local_venv_kaggle = BASE_DIR / ".venv" / "Scripts" / "kaggle.exe"
    if local_venv_kaggle.exists():
        return str(local_venv_kaggle)

    import shutil
    kaggle_bin = shutil.which("kaggle")
    if kaggle_bin:
        return kaggle_bin
        
    # Explicit fallbacks for systemd environments where PATH is stripped
    linux_fallback = Path("/home/dhruv/.local/bin/kaggle")
    if linux_fallback.exists():
        return str(linux_fallback)
        
    linux_global = Path("/usr/local/bin/kaggle")
    if linux_global.exists():
        return str(linux_global)

    return "kaggle"


def run_command(command: str):
    """Executes a shell command within the script directory."""
    result = subprocess.run(
        command,
        shell=True,
        text=True,
        capture_output=True,
        cwd=str(BASE_DIR)
    )
    if result.returncode != 0:
        print(f"[!] Command failed: {command}")
        print(f"[!] STDERR: {result.stderr.strip()}")
        return None
    return result.stdout.strip()


def build_automated_notebook() -> bool:
    """Reads llava.py and wraps it programmatically into a valid Jupyter Notebook."""
    if not SCRIPT_FILE.exists():
        print(f"[!] Error: {SCRIPT_FILE} not found locally.")
        return False

    with open(SCRIPT_FILE, "r", encoding="utf-8") as f:
        script_content = f.read()

    notebook_data = {
        "cells": [
            {
                "cell_type": "code",
                "execution_count": None,
                "metadata": {},
                "outputs": [],
                "source": [script_content]
            }
        ],
        "metadata": {
            "kernelspec": {
                "display_name": "Python 3",
                "language": "python",
                "name": "python3"
            },
            "language_info": {
                "name": "python"
            }
        },
        "nbformat": 4,
        "nbformat_minor": 2
    }

    with open(NOTEBOOK_FILE, "w", encoding="utf-8") as f:
        json.dump(notebook_data, f, indent=2)
    print("[+] Created automated_run.ipynb with embedded execution code.")
    return True


def update_env_file(ngrok_url: str):
    """Persists the newly discovered Ngrok URL to the .env file."""
    if not ENV_FILE.exists():
        return

    with open(ENV_FILE, "r", encoding="utf-8") as f:
        lines = f.readlines()

    updated = False
    new_lines = []
    for line in lines:
        if line.startswith("NGROK_BASE_URL="):
            new_lines.append(f"NGROK_BASE_URL={ngrok_url}\n")
            updated = True
        else:
            new_lines.append(line)

    if not updated:
        new_lines.append(f"\nNGROK_BASE_URL={ngrok_url}\n")

    with open(ENV_FILE, "w", encoding="utf-8") as f:
        f.writelines(new_lines)
    print(f"[+] Updated {ENV_FILE.name} with NGROK_BASE_URL={ngrok_url}")


def fetch_active_ngrok_url(timeout_seconds: int = 60) -> Optional[str]:
    """
    Connects to Kaggle log stream to automatically extract the live Ngrok tunnel URL.
    """
    try:
        try:
            import kaggle
        except ImportError:
            print("[!] ERROR: The 'kaggle' python package is not installed.")
            print("[!] Please run: pip install kaggle")
            return None
            
        os.environ["KAGGLE_CONFIG_DIR"] = str(BASE_DIR)
        kaggle.api.authenticate()

        print("[*] Scanning Kaggle execution stream for active Ngrok tunnel...")
        start_time = time.time()

        for event in kaggle.api.kernels_logs_stream(KERNEL_ID):
            if time.time() - start_time > timeout_seconds:
                print("[!] Log scanning timeout reached.")
                break

            text = event.get("data", "")
            match = re.search(r"https://[a-zA-Z0-9\-]+\.ngrok[a-zA-Z0-9\.\-]+", text)
            if match:
                url = match.group(0).rstrip("/.")
                return url

            if "SERVER ALIVE" in text:
                break
    except Exception as e:
        print(f"[!] Log Stream Notice: {e}")
    return None


def trigger_and_get_url(timeout_seconds: int = 120) -> Optional[str]:
    """
    Checks Kaggle status, triggers instance if necessary, and returns active Ngrok URL.
    """
    kaggle_bin = get_kaggle_cmd()
    
    # 1. Check if kernel is already running
    status_output = run_command(f'"{kaggle_bin}" kernels status {KERNEL_ID}')
    is_running = status_output and "running" in status_output.lower()

    if is_running:
        print(f"[*] Kaggle instance is already RUNNING ({KERNEL_ID}).")
    else:
        print("[*] Instance not running. Triggering Kaggle GPU cloud instance...")
        if not build_automated_notebook():
            return None
        
        # Wait for internet before pushing (Crucial for systemd boot)
        import socket
        import time
        print("[*] Waiting for internet connection...")
        for _ in range(30):
            try:
                socket.create_connection(("www.kaggle.com", 443), timeout=2)
                print("[+] Internet connection established.")
                break
            except OSError:
                time.sleep(2)
        else:
            print("[!] Warning: Could not reach Kaggle.com. Push may fail.")

        # Use resolved kaggle_bin to run the push command
        kaggle_bin = get_kaggle_cmd()
        push_cmd = f'"{kaggle_bin}" kernels push -p "{BASE_DIR}"'
        push_output = run_command(push_cmd)
        if not push_output:
            print(f"[!] Failed to push to Kaggle. Command: {push_cmd}")
            print("[!] Check credentials, kaggle.json, or internet connection.")
            return None
        print(f"[+] Kernel pushed: {push_output}")

    # 2. Extract Ngrok URL from log stream
    print("[*] Resolving active Ngrok endpoint from Kaggle logs...")
    url = fetch_active_ngrok_url(timeout_seconds=timeout_seconds)

    if url:
        print(f"\n[SUCCESS] Inferred Kaggle Ngrok Endpoint: {url}")
        update_env_file(url)
        return url
    else:
        print("[!] Could not automatically detect Ngrok URL from Kaggle stream.")
        print("    If the server was just launched, it may need ~45s to install packages and start the tunnel.")
        return None


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1].lower() == "stop":
        print("[*] Gracefully stopping Kaggle kernel via Ngrok kill-switch...")
        
        # Read the NGROK url from .env
        ngrok_url = None
        if ENV_FILE.exists():
            with open(ENV_FILE, "r") as f:
                for line in f:
                    if line.startswith("NGROK_BASE_URL="):
                        ngrok_url = line.split("=", 1)[1].strip()
                        break
        
        if ngrok_url:
            import requests
            try:
                print(f"[*] Sending kill ping to: {ngrok_url}/api/kill_dhruv")
                # Send exact same headers and method (POST) as ai_pipeline.py to guarantee it reaches Ollama
                resp = requests.post(
                    f"{ngrok_url}/api/kill_dhruv", 
                    headers={"Content-Type": "application/json", "ngrok-skip-browser-warning": "true"}, 
                    json={},
                    timeout=5
                )
                print(f"[+] Kill signal sent. Response status: {resp.status_code}")
            except Exception as e:
                print(f"[!] Kill request encountered an exception: {e}")
                print("[+] (This is often expected if the Kaggle server shuts down instantly and drops the connection).")
        else:
            print("[!] Could not find NGROK_BASE_URL in .env. Is the server running?")
    else:
        url = trigger_and_get_url()
        if url:
            print(f"\nActive Endpoint: {url}")
        else:
            print("\nNo endpoint resolved.")