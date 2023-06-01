# Extracts fields from a TAIP-PV message
# Returns longitude, latitude, speed(mph), and heading
# Returns empty list if msg parsing fails
def extract_fields(taip_pv):
    fields = []

    # Remove msg delimiters
    try:
        msg = taip_pv.split('>')
        msg = msg[1].split(';')
        msg = msg[0].split('RPV')
    except:
        print("BAD MSG")
        return fields

    #print("Full msg:",msg[1])
    # Pack msg into list for parsing
    msg = list(msg[1])


    # Separate fields
    #utc_time = ''.join(list(msg)[0:5])
    longitude = ''.join(list(msg)[5:13])
    latitude = ''.join(list(msg)[13:22])
    speed = ''.join(list(msg)[22:25])
    heading = ''.join(list(msg)[25:28])

    # print("Longitude: ",longitude)
    # print("Latitude: ", latitude)
    # print("Speed MPH: ", speed)
    # print("Heading: ", heading)

    # Conversion to decimal

    longitude = convert_to_decimal(longitude, msg_type="longitude")
    latitude = convert_to_decimal(latitude, msg_type="latitude")

    fields.append(longitude)
    fields.append(latitude)
    fields.append(speed)
    fields.append(heading)

    return fields

# Convert TAIP-PV latitude and longitude
# messages to decimal format
## TODO: Confirm that the GPS coordinates have the appropriate decimal place
# Previous implementation was off by a factor of 100
def convert_to_decimal(data, msg_type=""):
    if msg_type == "longitude":
        if data[0] == '+':
            msg = float(data[1:])/100.0
            return msg
        elif data[0] == '-':
            msg = -1.0*float(data[1:])/100.0
            return msg
        else:
            return "wrong data format"
    if msg_type == "latitude":
        if data[0] == '+':
            msg = float(data[1:])/100.0
            return msg
        elif data[0] == '-':
            msg = -1.0*float(data[1:])/100.0
            return msg
        else:
            return"wrong data format"
