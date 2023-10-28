-- switch_init.lua
station_config = {}
station_config.ssid = "what_the_hell"
station_config.pwd = "ChiLeChuan"

print('Setting up WIFI...')
wifi.setmode(wifi.STATION)
wifi.sta.config(station_config)
wifi.sta.connect()

gpio_pin = 3
switch_status = 'false'
gpio.mode(gpio_pin, gpio.OUTPUT)
http_server = net.createServer(net.TCP)

function set_attr(attr, value)
    if attr == "Power on" then
        if string.lower(value) == "true" then
            gpio.write(gpio_pin, gpio.HIGH)
            switch_status = 'true'
        else
            gpio.write(gpio_pin, gpio.LOW)
            switch_status = 'false'
        end
    end
    return get_attrs()
end

function get_attrs()
    temp_table = {type='switch', name='Power on', value=switch_status}
    attrs = {temp_table}

    ok, json = pcall(sjson.encode, attrs)
    if ok then
        print('current attrs: ' .. json)
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
            if not body_missing then print('[request start]\r\n' .. payload) end
            if string.find(payload, "GET /attrs") then
                response = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n"
                conn:send(response .. get_attrs())
            else
                -- POST/PUT request from Python requests, send the POST/PUT data in multiple chunks,
                -- Collect data packets until the size of HTTP body meets the Content-Length stated in header
                if payload:find("Content%-Length:") or body_missing then
                    if full_payload then full_payload = full_payload .. payload else full_payload = payload end
                    if (tonumber(string.match(full_payload, "%d+", full_payload:find("Content%-Length:")+16)) > #full_payload:sub(full_payload:find("\r\n\r\n", 1, true)+4, #full_payload)) then
                        body_missing = true
                        return
                    else
                        payload = full_payload
                        full_payload, body_missing = nil
                    end
                end
                
                if string.find(payload, "PUT /attr") then
                    local method, path, headers, body = string.match(payload, "([A-Z]+) (.-) HTTP/1.1\r\n(.+)\r\n\r\n(.+)")
                    local success, req_body = pcall(sjson.decode, body)
                    if success then
                        if req_body['attr'] and req_body['value'] then
                            response = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n"
                            conn:send(response .. set_attr(req_body['attr'], req_body['value']))
                        else
                            conn:send("HTTP/1.1 400 Bad Request\r\nInvalid request body\r\n\r\n")
                        end
                    else
                        conn:send("HTTP/1.1 400 Bad Request\r\nInvalid JSON data\r\n\r\n")
                    end
                else
                    conn:send("HTTP/1.1 404 Not Found\r\n\r\n")
                end
            end
            print('[request end]\n')
        end)
        
        conn:on("sent", function(conn)
            conn:close()
            collectgarbage()
        end)
    end)
end
