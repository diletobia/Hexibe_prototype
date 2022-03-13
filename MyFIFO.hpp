template <typename T, unsigned int len>
class MyFIFO {
public:
  MyFIFO();

  void push(T elem);
  T pop();

  bool empty();
  bool full();

  T ring_buffer[len];
  unsigned int wr_ptr;
  unsigned int rd_ptr;

};

template <typename T, unsigned int len>
MyFIFO<T,len>::MyFIFO() : wr_ptr(0), rd_ptr(0) {}

template <typename T, unsigned int len>
void MyFIFO<T,len>::push(T elem) {
  if (full())
    return;

  ring_buffer[wr_ptr] = elem;
  wr_ptr = (wr_ptr + 1) % len;
}

template <typename T, unsigned int len>
T MyFIFO<T,len>::pop() {
  T tmp;
  if (empty())
    return T();

  tmp = ring_buffer[rd_ptr];
  rd_ptr = (rd_ptr + 1) % len;

  return tmp;
}

template <typename T, unsigned int len>
bool MyFIFO<T,len>::full() {
  return ((wr_ptr + 1) % len) == rd_ptr;
}

template <typename T, unsigned int len>
bool MyFIFO<T,len>::empty() {
  return rd_ptr == wr_ptr;
}
