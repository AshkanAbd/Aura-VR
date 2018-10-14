collect_file = open('../data/hot_x_info.aura', 'r+')
write_file = open('../data/normalize_hot_x_info.aura', 'w+')
unique_x = []
for line in collect_file:
    array = line.split(' ')
    if array[0] not in unique_x:
        unique_x.append(array[0])
        write_file.write(line)
