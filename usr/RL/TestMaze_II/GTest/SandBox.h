class A;
class Notifier;

class Notifier {
public:
    Notifier();
    ~Notifier();
    void set_to_be_notified(A * p);
private:
    A * to_be_notified;
};

class A {
public:
    A();
    ~A();
    void set_to_be_notified(A * p) const;
    void notify_me();
private:
    Notifier * notifier;
    bool got_notified = false;
};
