import asyncio
import json
import threading
import subprocess
import os
import time
import websockets
from typing import List, Dict, Any, Optional

class LiveVisualizer:
    """
    WebSocket server for live trajectory visualization.
    """
    def __init__(self, host: str = "127.0.0.1", port: int = 8765) -> None:
        self.host = host
        self.port = port
        self.clients = set()
        self.loop = None
        self.thread = None
        self.stop_event = threading.Event()
        self._lock = threading.Lock()
        
    def start(self):
        """Start the WebSocket server in a separate thread, kicking any process already using the port."""
        print(f"Live visualizer: Initializing on {self.host}:{self.port}...")
        self._kill_process_on_port(self.port)
        self.thread = threading.Thread(target=self._run_server, daemon=True, name="VisualizerThread")
        self.thread.start()
        
    def _kill_process_on_port(self, port: int):
        """Find and kill any process listening on our port."""
        if os.name == 'nt':  # Windows
            try:
                # Get the process ID using the port
                cmd = f"netstat -ano | findstr :{port} | findstr LISTENING"
                output = subprocess.check_output(cmd, shell=True).decode()
                for line in output.splitlines():
                    if 'LISTENING' in line:
                        parts = line.strip().split()
                        if len(parts) >= 5:
                            pid = parts[-1]
                            if int(pid) != os.getpid():
                                print(f"Live visualizer: Reclaiming port {port} (killing PID {pid})")
                                subprocess.run(["taskkill", "/F", "/PID", pid], capture_output=True)
                                time.sleep(0.5) # Give OS time to release port
            except Exception:
                pass 
        else: # Linux / Mac
            try:
                cmd = f"fuser -k {port}/tcp"
                subprocess.run(cmd, shell=True, capture_output=True)
            except Exception:
                pass
        
    def _run_server(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        
        try:
            self.loop.run_until_complete(self._main())
        except Exception as e:
            print(f"Live visualizer: WebSocket server error: {e}")
        finally:
            self.loop.close()

    async def _main(self):
        try:
            async with websockets.serve(self._handler, self.host, self.port, reuse_address=True):
                print(f"Live visualizer: Server active at ws://{self.host}:{self.port}")
                print(f"Live visualizer: Open viz/index.html in your browser to view.")
                while not self.stop_event.is_set():
                    await asyncio.sleep(0.1)
        except Exception as e:
            print(f"Live visualizer: Failed to start server: {e}")

    async def _handler(self, websocket):
        # Ensure thread-safe modification of clients set
        with self._lock:
            # Kick any existing clients to ensure only one active connection (optional, but keeps it clean)
            if self.clients:
                for client in list(self.clients):
                    try:
                        await client.close(1001, "New client connected")
                    except:
                        pass
                self.clients.clear()
            self.clients.add(websocket)
            
        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    if data.get("type") == "regenerate":
                        print(f"Live visualizer: Received regenerate request for segments: {data.get('segments')}")
                except json.JSONDecodeError:
                    pass
        finally:
            with self._lock:
                self.clients.discard(websocket)

    def send_state(self, iteration: int, trajectory: List[Dict[str, Any]], phase: str = "solve"):
        """Send current solver state to all connected clients."""
        # Use a quick check for clients before scheduling to avoid overhead
        if not self.loop or not self.clients:
            return
            
        message = json.dumps({
            "type": "state",
            "iteration": iteration,
            "phase": phase,
            "trajectory": trajectory
        })
        
        # Schedule the broadcast in the event loop
        try:
            asyncio.run_coroutine_threadsafe(self._broadcast(message), self.loop)
        except RuntimeError:
            # Loop might be closing
            pass

    async def _broadcast(self, message):
        # Snapshoting clients to avoid modification issues
        current_clients = []
        with self._lock:
            current_clients = list(self.clients)
            
        if current_clients:
            # Send to all clients, ignore failures for individual clients
            for client in current_clients:
                try:
                    await client.send(message)
                except:
                    pass

    def stop(self):
        """Stop the server."""
        print("Live visualizer: Stopping...")
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=1.0)

# Global instance for easy access
_global_visualizer = None

def get_visualizer():
    global _global_visualizer
    if _global_visualizer is None:
        _global_visualizer = LiveVisualizer()
        _global_visualizer.start()
    return _global_visualizer
