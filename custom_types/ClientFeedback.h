

#ifndef CLIENTFEEDBACK_H
#define CLIENTFEEDBACK_H

#include <cereal/types/vector.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/tuple.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/archives/binary.hpp>


class ClientFeedback {
public:
    ClientFeedback(const long frame_id, const double fps)
        : frame_id(frame_id),
          fps(fps) {
    }

    ClientFeedback() = default;

    long frame_id;
    double fps;


    // Serialization method for Cereal
    template <class Archive>
    void serialize(Archive& archive)
    {
        archive(frame_id, fps);
    }
};



#endif //CLIENTFEEDBACK_H
