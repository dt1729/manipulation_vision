#!/usr/bin/env python3
"""
Deploy and validate the manipulation_vision Docker image on a remote GPU instance.

Supports AWS EC2, Lambda Labs, Vast.ai, and any SSH-accessible host (manual mode).

Usage:
  # Manual — works for any provider
  python3 deploy.py --host <IP> --key ~/.ssh/key.pem [--user ubuntu] [--port 22]

  # AWS EC2 (requires boto3, reads ~/.aws/credentials or env vars)
  python3 deploy.py --provider ec2 --instance-id i-0abc123 --key ~/.ssh/key.pem

  # Lambda Labs
  python3 deploy.py --provider lambda --instance-id <id> --api-key $LAMBDA_API_KEY --key ~/.ssh/lambda_key

  # Vast.ai (port resolved automatically — Vast uses random high ports)
  python3 deploy.py --provider vast --instance-id <id> --api-key $VAST_API_KEY --key ~/.ssh/vast_key
"""

import argparse
import os
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

try:
    import paramiko
except ImportError:
    sys.exit("paramiko not found — run: pip install paramiko")

# --------------------------------------------------------------------------- #
# Data types
# --------------------------------------------------------------------------- #

@dataclass
class SSHTarget:
    host: str
    port: int
    user: str
    key_path: Path


# --------------------------------------------------------------------------- #
# Provider resolvers
# --------------------------------------------------------------------------- #

class ProviderResolver:
    def resolve(self, args: argparse.Namespace) -> SSHTarget:
        raise NotImplementedError


class ManualResolver(ProviderResolver):
    def resolve(self, args):
        if not args.host:
            sys.exit("--host is required in manual mode")
        if not args.key:
            sys.exit("--key is required")
        return SSHTarget(
            host=args.host,
            port=args.port or 22,
            user=args.user or "ubuntu",
            key_path=Path(args.key).expanduser(),
        )


class EC2Resolver(ProviderResolver):
    def resolve(self, args):
        try:
            import boto3
        except ImportError:
            sys.exit("boto3 not found — run: pip install boto3")
        if not args.instance_id:
            sys.exit("--instance-id required for --provider ec2")
        if not args.key:
            sys.exit("--key required for --provider ec2")
        region = args.region or "us-east-1"
        ec2 = boto3.client("ec2", region_name=region)
        resp = ec2.describe_instances(InstanceIds=[args.instance_id])
        inst = resp["Reservations"][0]["Instances"][0]
        host = inst.get("PublicIpAddress") or inst.get("PublicDnsName")
        if not host:
            sys.exit(f"Instance {args.instance_id} has no public IP — is it running?")
        return SSHTarget(
            host=host,
            port=22,
            user=args.user or "ubuntu",
            key_path=Path(args.key).expanduser(),
        )


class LambdaResolver(ProviderResolver):
    def resolve(self, args):
        try:
            import requests
        except ImportError:
            sys.exit("requests not found — run: pip install requests")
        api_key = _resolve_api_key(args)
        if not args.instance_id:
            sys.exit("--instance-id required for --provider lambda")
        if not args.key:
            sys.exit("--key required for --provider lambda")
        url = f"https://cloud.lambdalabs.com/api/v1/instances/{args.instance_id}"
        resp = requests.get(url, headers={"Authorization": f"Bearer {api_key}"}, timeout=15)
        if resp.status_code != 200:
            sys.exit(f"Lambda API error {resp.status_code}: {resp.text}")
        data = resp.json().get("data", {})
        host = data.get("ip")
        if not host:
            sys.exit(f"No IP in Lambda response: {data}")
        return SSHTarget(
            host=host,
            port=22,
            user=args.user or "ubuntu",
            key_path=Path(args.key).expanduser(),
        )


class VastResolver(ProviderResolver):
    def resolve(self, args):
        try:
            import requests
        except ImportError:
            sys.exit("requests not found — run: pip install requests")
        api_key = _resolve_api_key(args)
        if not args.instance_id:
            sys.exit("--instance-id required for --provider vast")
        if not args.key:
            sys.exit("--key required for --provider vast")
        url = f"https://console.vast.ai/api/v0/instances/{args.instance_id}/"
        resp = requests.get(url, params={"api_key": api_key}, timeout=15)
        if resp.status_code != 200:
            sys.exit(f"Vast API error {resp.status_code}: {resp.text}")
        data = resp.json()
        host = data.get("ssh_host") or data.get("public_ipaddr")
        port = data.get("ssh_port") or 22
        if not host:
            sys.exit(f"No SSH host in Vast response: {data}")
        return SSHTarget(
            host=host,
            port=int(port),
            user=args.user or "root",
            key_path=Path(args.key).expanduser(),
        )


RESOLVERS = {
    None:     ManualResolver(),
    "ec2":    EC2Resolver(),
    "lambda": LambdaResolver(),
    "vast":   VastResolver(),
}


def _resolve_api_key(args) -> str:
    if args.api_key:
        return args.api_key
    if args.api_key_env:
        val = os.environ.get(args.api_key_env)
        if not val:
            sys.exit(f"Environment variable {args.api_key_env!r} is not set")
        return val
    sys.exit("--api-key or --api-key-env required for this provider")


# --------------------------------------------------------------------------- #
# SSH helpers
# --------------------------------------------------------------------------- #

def _connect(target: SSHTarget) -> paramiko.SSHClient:
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(
        hostname=target.host,
        port=target.port,
        username=target.user,
        key_filename=str(target.key_path),
        timeout=30,
    )
    return client


def _run(client: paramiko.SSHClient, cmd: str, timeout: int = 300) -> tuple[int, str, str]:
    """Run a command over SSH. Returns (exit_code, stdout, stderr)."""
    _, stdout, stderr = client.exec_command(cmd, timeout=timeout)
    out = stdout.read().decode(errors="replace")
    err = stderr.read().decode(errors="replace")
    code = stdout.channel.recv_exit_status()
    return code, out, err


def _run_stream(client: paramiko.SSHClient, cmd: str, label: str, timeout: int = 1800):
    """Run a command and stream stdout line-by-line. Returns exit code."""
    transport = client.get_transport()
    channel = transport.open_session()
    channel.set_combine_stderr(True)
    channel.exec_command(cmd)
    channel.settimeout(timeout)
    buf = b""
    while not channel.exit_status_ready():
        if channel.recv_ready():
            buf += channel.recv(4096)
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                print(f"  {line.decode(errors='replace')}")
    # drain remaining
    while channel.recv_ready():
        buf += channel.recv(4096)
    for line in buf.split(b"\n"):
        if line.strip():
            print(f"  {line.decode(errors='replace')}")
    return channel.recv_exit_status()


# --------------------------------------------------------------------------- #
# Deploy steps
# --------------------------------------------------------------------------- #

def preflight(client: paramiko.SSHClient, target: SSHTarget, provider: str):
    print(f"\n[preflight]  provider={provider or 'manual'}  host={target.host}  "
          f"port={target.port}  user={target.user}")

    code, out, _ = _run(client, "docker --version 2>/dev/null")
    if code != 0:
        print("  Docker not found — installing...")
        # detect distro
        _, out2, _ = _run(client, "cat /etc/os-release")
        pkg_mgr = "apt-get" if "ubuntu" in out2.lower() or "debian" in out2.lower() else "yum"
        if pkg_mgr == "apt-get":
            install_cmd = (
                "sudo apt-get update -qq && "
                "sudo apt-get install -y docker.io && "
                "sudo systemctl start docker"
            )
        else:
            install_cmd = (
                "sudo yum install -y docker && "
                "sudo systemctl start docker"
            )
        rc = _run_stream(client, install_cmd, "install docker")
        if rc != 0:
            sys.exit("Docker installation failed")
        _, out, _ = _run(client, "docker --version")

    # ensure user can run docker without sudo
    _run(client, f"sudo usermod -aG docker {target.user} 2>/dev/null || true")
    _run(client, "sudo systemctl start docker 2>/dev/null || true")

    print(f"  {out.strip()}")


def rsync_repo(target: SSHTarget):
    print("\n[rsync]      syncing repo → ~/manipulation_vision/ ...")
    repo_root = Path(__file__).parent.parent.parent  # scripts/deploy/deploy.py → repo root

    excludes = [
        "--exclude=.git",
        "--exclude=__pycache__",
        "--exclude=*.pyc",
        "--exclude=*.egg-info",
        "--exclude=scripts/*.mp4",
        "--exclude=scripts/*.npy",
        "--exclude=includes/mujoco-py",
    ]
    ssh_cmd = (
        f"ssh -p {target.port} -i {target.key_path} "
        f"-o StrictHostKeyChecking=no -o BatchMode=yes"
    )
    cmd = [
        "rsync", "-az", "--progress",
        *excludes,
        "-e", ssh_cmd,
        f"{repo_root}/",
        f"{target.user}@{target.host}:~/manipulation_vision/",
    ]
    t0 = time.time()
    result = subprocess.run(cmd, check=False)
    if result.returncode != 0:
        sys.exit("rsync failed")
    print(f"  done ({time.time() - t0:.0f}s)")


def docker_build(client: paramiko.SSHClient, tag: str):
    print(f"\n[build]      docker build {tag} ...")
    t0 = time.time()
    rc = _run_stream(
        client,
        f"cd ~/manipulation_vision && docker build -t {tag} .",
        "docker build",
    )
    if rc != 0:
        sys.exit("Docker build failed")
    _, size_out, _ = _run(client, f"docker image inspect {tag} --format='{{{{.Size}}}}'")
    try:
        size_gb = int(size_out.strip().strip("'")) / 1e9
        print(f"  done ({(time.time()-t0)/60:.1f}m, {size_gb:.1f}GB)")
    except ValueError:
        print(f"  done ({(time.time()-t0)/60:.1f}m)")


def validate(client: paramiko.SSHClient, tag: str) -> bool:
    tests = [
        (
            "import smoke test",
            "python3 -c \"import mujoco, robocasa, robosuite; print('mujoco', mujoco.__version__)\"",
            30,
        ),
        (
            "tactile demo",
            "python3 scripts/orange_individual_touch.py",
            60,
        ),
        (
            "kitchen demo",
            "python3 -m robocasa.demos.demo_kitchen_states --task PnPCounterToCab --robot PiperOmron",
            120,
        ),
        (
            "joint cmd",
            "python3 scripts/send_joint_cmd.py --settle 0 --joints 0 0.5 -1.0 0 0.8 0",
            60,
        ),
    ]

    print()
    results = []
    all_passed = True
    for name, cmd, timeout in tests:
        docker_cmd = (
            f"docker run --rm -e MUJOCO_GL=osmesa "
            f"-w /opt/manipulation_vision {tag} {cmd}"
        )
        t0 = time.time()
        code, out, err = _run(client, docker_cmd, timeout=timeout + 10)
        elapsed = time.time() - t0
        passed = code == 0
        status = "PASS" if passed else "FAIL"
        print(f"[validate]   {name:<30} {status}  {elapsed:5.1f}s")
        if not passed:
            all_passed = False
            combined = (out + err).strip()
            tail = "\n".join(combined.splitlines()[-50:])
            print(f"\n  --- last output ---\n{tail}\n  ---\n")
        results.append((name, passed))

    print()
    if all_passed:
        print(f"All {len(tests)} tests passed.")
    else:
        failed = [n for n, p in results if not p]
        print(f"{len(failed)}/{len(tests)} tests FAILED: {', '.join(failed)}")
    return all_passed


# --------------------------------------------------------------------------- #
# Entry point
# --------------------------------------------------------------------------- #

def main():
    parser = argparse.ArgumentParser(
        description="Deploy and validate manipulation_vision Docker image on a remote instance."
    )

    # Connection — manual mode
    parser.add_argument("--host",        help="Remote host IP or DNS (manual mode)")
    parser.add_argument("--port",        type=int, default=None, help="SSH port (default: 22)")
    parser.add_argument("--user",        default=None, help="SSH username")
    parser.add_argument("--key",         help="Path to SSH private key (.pem)")

    # Provider auto-resolve
    parser.add_argument("--provider",    choices=["ec2", "lambda", "vast"],
                        help="Cloud provider for auto-resolving connection details")
    parser.add_argument("--instance-id", dest="instance_id", help="Instance ID")
    parser.add_argument("--region",      default=None, help="AWS region (ec2 only)")
    parser.add_argument("--api-key",     dest="api_key", default=None,
                        help="Provider API key (lambda/vast)")
    parser.add_argument("--api-key-env", dest="api_key_env", default=None,
                        help="Env var name containing the API key")

    # Deploy flags
    parser.add_argument("--tag",         default="manipulation_vision:latest",
                        help="Docker image tag to build/run")
    parser.add_argument("--skip-rsync",  action="store_true",
                        help="Skip rsync (repo already on remote)")
    parser.add_argument("--skip-build",  action="store_true",
                        help="Skip docker build (image already on remote)")

    args = parser.parse_args()

    # mutual exclusion: provider vs manual host
    if args.provider and args.host:
        parser.error("--provider and --host are mutually exclusive")

    resolver = RESOLVERS[args.provider]
    target = resolver.resolve(args)

    print(f"Connecting to {target.user}@{target.host}:{target.port} ...")
    client = _connect(target)

    try:
        preflight(client, target, args.provider)

        if not args.skip_rsync:
            rsync_repo(target)

        if not args.skip_build:
            docker_build(client, args.tag)

        passed = validate(client, args.tag)
    finally:
        client.close()

    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
