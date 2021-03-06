#ifndef JOIN_THREADS_H
#define JOIN_THREADS_H

#include<vector>
#include<thread>

class join_threads{
private:
  std::vector<std::thread>& threads;

public:
  explicit join_threads(std::vector<std::thread>& threads_):
    threads(threads_){}   // 冒号赋值法，将threads_赋值给threads

  ~join_threads(){
    for(unsigned long i = 0; i < threads.size(); ++i){
        if(threads[i].joinable())
          threads[i].join();
      }
  }
};

#endif // JOIN_THREADS_H
