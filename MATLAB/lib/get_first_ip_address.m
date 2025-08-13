function ip = get_first_ip_address()
    [status, result] = system('hostname -I');
    if status == 0 && ~isempty(strtrim(result))
        ips = strsplit(strtrim(result));
        ip = ips{1};
    else
        error('Could not detect IP. Run "hostname -I" to check your network setup.');
    end
end