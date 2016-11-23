
int main()
{
    struct hw_module_t module;
    char id;
    struct hw_device_t* p_hw_device_t;
    sensors_poll_context_t *dev;
    int i;
#define MAX_EVENTS_READ 26
    sensors_event_t events[MAX_EVENTS_READ];
    int msg_cnt = 0;

    sensors_module_methods.open(&module, &id, &p_hw_device_t);

    for (i = 0; i < sensorsNum; ++i) {
        printf("sSensorList[%d].name = %s, handle = %d \n",
                i, sSensorList[i].name, sSensorList[i].handle);
    }
    printf("sensorsNum = %d \n", sensorsNum);

    dev = (sensors_poll_context_t *)p_hw_device_t;
    for (i = 0; i < sensorsNum; ++i) {
        dev->device.activate((sensors_poll_device_t *)dev, sSensorList[i].handle, 0);
    }

    for (i = 0; i < sensorsNum; ++i) {
        dev->device.batch((sensors_poll_device_1 *)dev, sSensorList[i].handle, 0, 5000000, 0);
        dev->device.activate((sensors_poll_device_t *)dev, sSensorList[i].handle, 1);
    }

    while(1)
    {
        msg_cnt = dev->device.poll((sensors_poll_device_t *)dev, events, MAX_EVENTS_READ);
        for (i = 0; i < msg_cnt; ++i) {

            if(META_DATA_VERSION == events[i].version &&
                    (int)0xFFFFFFFF == events[i].meta_data.sensor)
            {
                printf("ending polling...\n");
                goto END;
            }else{
                printf("get event, sensor id = %d\n", events[i].sensor);
            }
        }

        printf("==========================\n\n");
    }

END:

    for (i = 0; i < sensorsNum; ++i) {
        dev->device.activate((sensors_poll_device_t *)dev, sSensorList[i].handle, 0);
    }

    delete(dev);

    return 0;
}
