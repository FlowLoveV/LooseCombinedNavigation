# yaml-cpp使用方法记录

## 基本使用
```
YAML::Node config = YAML::LoadFile("../config.yaml");
cout << "name:" << config["name"].as<string>() << endl;
cout << "sex:" << config["sex"].as<string>() << endl;
cout << "age:" << config["age"].as<int>() << endl;
cout << config["skills"]["c++"].as<string>() << endl;
return 0;
```