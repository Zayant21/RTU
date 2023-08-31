from flask import Flask, render_template, jsonify
from flask_mail import Mail, Message
import socket
import time

app = Flask(__name__)

UDP_IP = "192.168.16.177"
UDP_PORT = 8888
Local_IP = "192.168.16.24"
Local_Port = 9291

expected_start_of_message = 0xAA
expected_function_code = 0xFC

alarm_descriptions = {
    0x00: "Comfortable",
    0x08: "Major Over",
    0x0C: "Minor Over",
    0x02: "Minor Under",
    0x01: "Major Under"
}

prev_alarm_state = 0x00

app.config['MAIL_SERVER'] = 'localhost'
app.config['MAIL_PORT'] = 1122
app.config['MAIL_USE_TLS'] = False
app.config['MAIL_USE_SSL'] = False
app.config['MAIL_USERNAME'] = 'zayan'
app.config['MAIL_PASSWORD'] = 'your_password'
app.config['MAIL_DEFAULT_SENDER'] = 'zayan@example.com'  # Replace with your email

mail = Mail(app)


def send_notification_email(address, alarm):
    alarm_description = alarm_descriptions.get(alarm, "Unknown Alarm")
    
    msg = Message("Alarm Notification",
                  recipients=["recipient@example.com"])  # Replace with recipient's email
    msg.body = f"Alarm received from device {address}. Alarm state: {alarm_description}"
    
    try:
        mail.send(msg)
        print("Notification email sent")
    except Exception as e:
        print("Failed to send notification email:", str(e))




def calculate_bch(data):
    nBCHpoly = 0xB8
    fBCHpoly = 0xFF
    bch = data

    for j in range(8):
        if bch & 1:
            bch = (bch >> 1) ^ nBCHpoly
        else:
            bch >>= 1
    bch ^= fBCHpoly
    return bch


def create_dcp_request(address, opcode):
    header = bytes.fromhex("AA")
    function = bytes.fromhex("FC")
    opcode_byte = bytes.fromhex(opcode)
    polled_addressess = bytes([address])
    checksum = calculate_bch(int.from_bytes(bytes.fromhex(opcode), byteorder='big'))
    checksum_byte = bytes([checksum])

    dcp_request = header + function + polled_addressess + opcode_byte + checksum_byte
    return dcp_request


def send_dcp_req(address, opcode):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    data_packet = create_dcp_request(address, opcode)
    sock.sendto(data_packet, (UDP_IP, UDP_PORT))
    print("DCP request sent: ", data_packet)
    sock.close()


def process_response(response, polled_addresses):
    global prev_alarm_state
    if len(response) >= 5:
        received_start_of_message = response[0]
        received_function_code = response[1]
        received_address = response[2]
        received_temperature = response[3]
        received_alarm = response[4]
        received_bch = response[5]

        if (received_start_of_message == expected_start_of_message and
                received_function_code == expected_function_code and
                received_address in polled_addresses):

            received_temperature_bch = calculate_bch(received_temperature ^ received_alarm)

            if received_bch == received_temperature_bch:
                print("Received data: Address={}, Temperature={}, BCH={}".format(
                    hex(received_address), received_temperature, hex(received_bch)))

                if received_alarm != prev_alarm_state:
                    print("alarm send")
                    send_notification_email(hex(received_address), received_alarm)
                    prev_alarm_state = received_alarm
                return [received_temperature, received_alarm, received_address, True]

            else:
                print("Invalid BCH for temperature")
        else:
            print("Invalid header, function code, or address")
    else:
        print("Invalid response format")



@app.route('/')
def live_temperature_page():
    data_list = get_current_data()
    return render_template("temperature.html")

@app.route('/data')
def live_data():
    data_list = get_current_data()
    data = {
        'data_list': data_list
    }
    return jsonify(data)



def get_current_data():
    polled_addresses = [0x43, 0x44, 0x45, 0x66]
    opcode = "03"
    for address in polled_addresses:
        send_dcp_req(address, opcode)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((Local_IP, Local_Port))



    data_list = []

    data, addr = sock.recvfrom(1024)
    temperature = process_response(data, polled_addresses)
    if temperature is not None:
        data_list.append(temperature)

    for address in polled_addresses:
        if all(entry[2] != address for entry in data_list):
            data_list.append([None, None, address, False])
    return data_list


if __name__ == '__main__':
    app.run(debug=True)
