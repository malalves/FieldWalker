// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: Evolve.proto

#ifndef PROTOBUF_Evolve_2eproto__INCLUDED
#define PROTOBUF_Evolve_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 2006000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 2006001 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)

namespace evolve_robots_msgs {
namespace msgs {

// Internal implementation detail -- do not call these.
void  protobuf_AddDesc_Evolve_2eproto();
void protobuf_AssignDesc_Evolve_2eproto();
void protobuf_ShutdownFile_Evolve_2eproto();

class Evolve;

// ===================================================================

class Evolve : public ::google::protobuf::Message {
 public:
  Evolve();
  virtual ~Evolve();

  Evolve(const Evolve& from);

  inline Evolve& operator=(const Evolve& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _unknown_fields_;
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return &_unknown_fields_;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const Evolve& default_instance();

  void Swap(Evolve* other);

  // implements Message ----------------------------------------------

  Evolve* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Evolve& from);
  void MergeFrom(const Evolve& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const;
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  public:
  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // required int32 index = 1;
  inline bool has_index() const;
  inline void clear_index();
  static const int kIndexFieldNumber = 1;
  inline ::google::protobuf::int32 index() const;
  inline void set_index(::google::protobuf::int32 value);

  // required double time = 2;
  inline bool has_time() const;
  inline void clear_time();
  static const int kTimeFieldNumber = 2;
  inline double time() const;
  inline void set_time(double value);

  // @@protoc_insertion_point(class_scope:evolve_robots_msgs.msgs.Evolve)
 private:
  inline void set_has_index();
  inline void clear_has_index();
  inline void set_has_time();
  inline void clear_has_time();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  double time_;
  ::google::protobuf::int32 index_;
  friend void  protobuf_AddDesc_Evolve_2eproto();
  friend void protobuf_AssignDesc_Evolve_2eproto();
  friend void protobuf_ShutdownFile_Evolve_2eproto();

  void InitAsDefaultInstance();
  static Evolve* default_instance_;
};
// ===================================================================


// ===================================================================

// Evolve

// required int32 index = 1;
inline bool Evolve::has_index() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Evolve::set_has_index() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Evolve::clear_has_index() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void Evolve::clear_index() {
  index_ = 0;
  clear_has_index();
}
inline ::google::protobuf::int32 Evolve::index() const {
  // @@protoc_insertion_point(field_get:evolve_robots_msgs.msgs.Evolve.index)
  return index_;
}
inline void Evolve::set_index(::google::protobuf::int32 value) {
  set_has_index();
  index_ = value;
  // @@protoc_insertion_point(field_set:evolve_robots_msgs.msgs.Evolve.index)
}

// required double time = 2;
inline bool Evolve::has_time() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void Evolve::set_has_time() {
  _has_bits_[0] |= 0x00000002u;
}
inline void Evolve::clear_has_time() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void Evolve::clear_time() {
  time_ = 0;
  clear_has_time();
}
inline double Evolve::time() const {
  // @@protoc_insertion_point(field_get:evolve_robots_msgs.msgs.Evolve.time)
  return time_;
}
inline void Evolve::set_time(double value) {
  set_has_time();
  time_ = value;
  // @@protoc_insertion_point(field_set:evolve_robots_msgs.msgs.Evolve.time)
}


// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace evolve_robots_msgs

#ifndef SWIG
namespace google {
namespace protobuf {


}  // namespace google
}  // namespace protobuf
#endif  // SWIG

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_Evolve_2eproto__INCLUDED
