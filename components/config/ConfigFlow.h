#ifndef CONFIGFLOW_H
#define CONFIGFLOW_H
#include <NanoAkka.h>
class ConfigStore
{
public:
    static void init();
    bool load(const char *name, void *value, uint32_t length);
    bool save(const char *name, void *value, uint32_t length);
    bool load(const char *name, std::string* value,uint32_t length);
    bool save(const char *name, std::string* value,uint32_t length);
};

template <class T> class ConfigFlow : public Flow<T, T>, public ConfigStore
{
    std::string _name;
    T _value;
    bool ready=false;

public:
    ConfigFlow(const char *name, T defaultValue)
        : _name(name), _value(defaultValue)
    {
        _value = defaultValue;
    }

    void operator=(T t)
    {
        on(t);
    }

    void lazyLoad()
    {
        if ( ready ) return;
        init();
        if (load(_name.c_str(), &_value, sizeof(T))) {
            INFO(" Config load %s ", _name.c_str());
        } else {
            INFO(" Config default %s ", _name.c_str());
        }
        ready=true;
    }


    void on(const T &value)
    {
        _value = value;
        if ( save(_name.c_str(), &_value, sizeof(T)) ) {
            request();
        } else {
            WARN("config save failed : %s",_name.c_str());
        }
    }
    void request()
    {
        lazyLoad();
        this->emit(_value);
    }

    inline T operator()()
    {
        lazyLoad();
        return _value;
    }
};

#endif // CONFIGFLOW_H
