
Arduino en la raspi

## Instalacion
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
sudo mv bin/arduino-cli /usr/local/bin/

arduino-cli config init

arduino-cli core update-index
arduino-cli core install arduino:avr

arduino-cli lib install "AccelStepper"

arduino-cli board list

## Actualizar codigo

arduino-cli compile --fqbn arduino:avr:mega Stepper

arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:mega Stepper

## Comandos útiles

- scp mask.jpg mathigatti@192.168.1.29:~

- run 
    source ~/caricias/bin/activate

    python control.py
    uvicorn move:app --host 0.0.0.0 --port 8000

    ngrok http http://localhost:8000 (opcional)

- sudo journalctl -u caricias-move.service -f
