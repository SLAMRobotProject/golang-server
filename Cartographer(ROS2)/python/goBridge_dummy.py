import asyncio
import websockets
import json
import logging

logging.basicConfig(level=logging.INFO)
log = logging.getLogger(__name__)

async def websocket_handler(websocket):
    """
    Handles incoming WebSocket messages.
    This dummy version just logs the client connection
    and prints any message it receives.
    """
    try:
        async for message in websocket:
            try:
                data = json.loads(message)
                log.info("--- Received Valid JSON ---")
                
                # Pretty-print the JSON data
                print(json.dumps(data, indent=2))
                print();

            except json.JSONDecodeError:
                log.error('Received malformed JSON! Printing raw message:')
                print(message)
            except Exception as e:
                log.error(f'Error processing message: {e}')
    
    except websockets.exceptions.ConnectionClosed:
        log.warning(f'WebSocket client disconnected.')
    except Exception as e:
        log.error(f'An unexpected error occurred: {e}')

async def main():
    """ Runs the dummy WebSocket server """
    
    host = "0.0.0.0"
    port = 8765
    
    log.info(f'Starting dummy WebSocket server on ws://{host}:{port}')
    
    async with websockets.serve(websocket_handler, host, port):
        await asyncio.Future()  # Run forever

if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        log.info('Server shutting down.')