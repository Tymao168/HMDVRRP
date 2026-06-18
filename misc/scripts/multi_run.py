from scripts.batch_main import main

file_list = ["RM-d2-n3-k1-p2.json","RM-d2-n4-k1-p2.json","RM-d2-n5-k1-p2.json","RM-d2-n10-k1-p2.json","M-d3-n6-k1-p2.json","RM-d3-n6-k1-p2.json"]
for file in file_list:
    main(instance_name=file)
