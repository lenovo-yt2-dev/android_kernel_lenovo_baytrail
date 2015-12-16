struct hall_switch_data{
    struct input_dev *input_dev;
    struct work_struct hall_work;
    struct workqueue_struct *hall_workqueue;
    int hall_irq;
    int hall_gpio_val;
};
