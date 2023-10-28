-- dht11_init.lua
station_config = {}
station_config.ssid = "what_the_hell"
station_config.pwd = "ChiLeChuan"

print('Setting up WIFI...')
wifi.setmode(wifi.STATION)
wifi.sta.config(station_config)
wifi.sta.connect()

dht11_pin = 1
http_server = net.createServer(net.TCP)

function get_attrs()
    local status, temp, humi , temp_dec, humi_dec = dht.read11(dht11_pin)
    temp_table = {type='text', name='temp', value=temp..'.'..temp_dec.."'C", read_only=true}
    humi_table = {type='text', name='humidity', value=humi..'.'..humi_dec..'%', read_only=true}
    attrs = {temp_table, humi_table}

    ok, json = pcall(sjson.encode, attrs)
    if ok then
        print('attrs: ' .. json)
        return json
    else
        print("failed to encode!")
        return '[]'
    end
end

tmr.alarm(1, 1000, tmr.ALARM_AUTO, function()
    if wifi.sta.getip() == nil then
        print('Waiting for IP...')
    else
        print('IP is ' .. wifi.sta.getip() .. ', MAC is ' .. wifi.sta.getmac())
        tmr.stop(1)
    end
end)

if http_server then
    http_server:listen(8880, function(conn)
        conn:on("receive", function(conn, payload)
            print('req body: ' .. payload)
            if string.find(payload, "GET /attrs") then
                response = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n"
                conn:send(response .. get_attrs())
            else
                conn:send("HTTP/1.1 404 Not Found\r\n\r\n")
            end
        end)
        conn:on("sent", function(conn)
            conn:close() end)
    end)
end
