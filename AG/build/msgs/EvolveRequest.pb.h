// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: EvolveRequest.proto

#ifndef PROTOBUF_EvolveRequest_2eproto__INCLUDED
#define PROTOBUF_EvolveRequest_2eproto__INCLUDED

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
void  protobuf_AddDesc_EvolveRequest_2eproto();
void protobuf_AssignDesc_EvolveRequest_2eproto();
void protobuf_ShutdownFile_EvolveRequest_2eproto();

class EvolveRequest;

// ===================================================================

class EvolveRequest : public ::google::protobuf::Message {
 public:
  EvolveRequest();
  virtual ~EvolveRequest();

  EvolveRequest(const EvolveRequest& from);

  inline EvolveRequest& operator=(const EvolveRequest& from) {
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
  static const EvolveRequest& default_instance();

  void Swap(EvolveRequest* other);

  // implements Message ----------------------------------------------

  EvolveRequest* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const EvolveRequest& from);
  void MergeFrom(const EvolveRequest& from);
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

  // repeated int32 road = 1;
  inline int road_size() const;
  inline void clear_road();
  static const int kRoadFieldNumber = 1;
  inline ::google::protobuf::int32 road(int index) const;
  inline void set_road(int index, ::google::protobuf::int32 value);
  inline void add_road(::google::protobuf::int32 value);
  inline const ::google::protobuf::RepeatedField< ::google::protobuf::int32 >&
      road() const;
  inline ::google::protobuf::RepeatedField< ::google::protobuf::int32 >*
      mutable_road();

  // repeated double speeds = 2;
  inline int speeds_size() const;
  inline void clear_speeds();
  static const int kSpeedsFieldNumber = 2;
  inline double speeds(int index) const;
  inline void set_speeds(int index, double value);
  inline void add_speeds(double value);
  inline const ::google::protobuf::RepeatedField< double >&
      speeds() const;
  inline ::google::protobuf::RepeatedField< double >*
      mutable_speeds();

  // repeated double carrot = 3;
  inline int carrot_size() const;
  inline void clear_carrot();
  static const int kCarrotFieldNumber = 3;
  inline double carrot(int index) const;
  inline void set_carrot(int index, double value);
  inline void add_carrot(double value);
  inline const ::google::protobuf::RepeatedField< double >&
      carrot() const;
  inline ::google::protobuf::RepeatedField< double >*
      mutable_carrot();

  // required int32 index = 4;
  inline bool has_index() const;
  inline void clear_index();
  static const int kIndexFieldNumber = 4;
  inline ::google::protobuf::int32 index() const;
  inline void set_index(::google::protobuf::int32 value);

  // @@protoc_insertion_point(class_scope:evolve_robots_msgs.msgs.EvolveRequest)
 private:
  inline void set_has_index();
  inline void clear_has_index();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::google::protobuf::RepeatedField< ::google::protobuf::int32 > road_;
  ::google::protobuf::RepeatedField< double > speeds_;
  ::google::protobuf::RepeatedField< double > carrot_;
  ::google::protobuf::int32 index_;
  friend void  protobuf_AddDesc_EvolveRequest_2eproto();
  friend void protobuf_AssignDesc_EvolveRequest_2eproto();
  friend void protobuf_ShutdownFile_EvolveRequest_2eproto();

  void InitAsDefaultInstance();
  static EvolveRequest* default_instance_;
};
// ===================================================================


// ===================================================================

// EvolveRequest

// repeated int32 road = 1;
inline int EvolveRequest::road_size() const {
  return road_.size();
}
inline void EvolveRequest::clear_road() {
  road_.Clear();
}
inline ::google::protobuf::int32 EvolveRequest::road(int index) const {
  // @@protoc_insertion_point(field_get:evolve_robots_msgs.msgs.EvolveRequest.road)
  return road_.Get(index);
}
inline void EvolveRequest::set_road(int index, ::google::protobuf::int32 value) {
  road_.Set(index, value);
  // @@protoc_insertion_point(field_set:evolve_robots_msgs.msgs.EvolveRequest.road)
}
inline void EvolveRequest::add_road(::google::protobuf::int32 value) {
  road_.Add(value);
  // @@protoc_insertion_point(field_add:evolve_robots_msgs.msgs.EvolveRequest.road)
}
inline const ::google::protobuf::RepeatedField< ::google::protobuf::int32 >&
EvolveRequest::road() const {
  // @@protoc_insertion_point(field_list:evolve_robots_msgs.msgs.EvolveRequest.road)
  return road_;
}
inline ::google::protobuf::RepeatedField< ::google::protobuf::int32 >*
EvolveRequest::mutable_road() {
  // @@protoc_insertion_point(field_mutable_list:evolve_robots_msgs.msgs.EvolveRequest.road)
  return &road_;
}

// repeated double speeds = 2;
inline int EvolveRequest::speeds_size() const {
  return speeds_.size();
}
inline void EvolveRequest::clear_speeds() {
  speeds_.Clear();
}
inline double EvolveRequest::speeds(int index) const {
  // @@protoc_insertion_point(field_get:evolve_robots_msgs.msgs.EvolveRequest.speeds)
  return speeds_.Get(index);
}
inline void EvolveRequest::set_speeds(int index, double value) {
  speeds_.Set(index, value);
  // @@protoc_insertion_point(field_set:evolve_robots_msgs.msgs.EvolveRequest.speeds)
}
inline void EvolveRequest::add_speeds(double value) {
  speeds_.Add(value);
  // @@protoc_insertion_point(field_add:evolve_robots_msgs.msgs.EvolveRequest.speeds)
}
inline const ::google::protobuf::RepeatedField< double >&
EvolveRequest::speeds() const {
  // @@protoc_insertion_point(field_list:evolve_robots_msgs.msgs.EvolveRequest.speeds)
  return speeds_;
}
inline ::google::protobuf::RepeatedField< double >*
EvolveRequest::mutable_speeds() {
  // @@protoc_insertion_point(field_mutable_list:evolve_robots_msgs.msgs.EvolveRequest.speeds)
  return &speeds_;
}

// repeated double carrot = 3;
inline int EvolveRequest::carrot_size() const {
  return carrot_.size();
}
inline void EvolveRequest::clear_carrot() {
  carrot_.Clear();
}
inline double EvolveRequest::carrot(int index) const {
  // @@protoc_insertion_point(field_get:evolve_robots_msgs.msgs.EvolveRequest.carrot)
  return carrot_.Get(index);
}
inline void EvolveRequest::set_carrot(int index, double value) {
  carrot_.Set(index, value);
  // @@protoc_insertion_point(field_set:evolve_robots_msgs.msgs.EvolveRequest.carrot)
}
inline void EvolveRequest::add_carrot(double value) {
  carrot_.Add(value);
  // @@protoc_insertion_point(field_add:evolve_robots_msgs.msgs.EvolveRequest.carrot)
}
inline const ::google::protobuf::RepeatedField< double >&
EvolveRequest::carrot() const {
  // @@protoc_insertion_point(field_list:evolve_robots_msgs.msgs.EvolveRequest.carrot)
  return carrot_;
}
inline ::google::protobuf::RepeatedField< double >*
EvolveRequest::mutable_carrot() {
  // @@protoc_insertion_point(field_mutable_list:evolve_robots_msgs.msgs.EvolveRequest.carrot)
  return &carrot_;
}

// required int32 index = 4;
inline bool EvolveRequest::has_index() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void EvolveRequest::set_has_index() {
  _has_bits_[0] |= 0x00000008u;
}
inline void EvolveRequest::clear_has_index() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void EvolveRequest::clear_index() {
  index_ = 0;
  clear_has_index();
}
inline ::google::protobuf::int32 EvolveRequest::index() const {
  // @@protoc_insertion_point(field_get:evolve_robots_msgs.msgs.EvolveRequest.index)
  return index_;
}
inline void EvolveRequest::set_index(::google::protobuf::int32 value) {
  set_has_index();
  index_ = value;
  // @@protoc_insertion_point(field_set:evolve_robots_msgs.msgs.EvolveRequest.index)
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

#endif  // PROTOBUF_EvolveRequest_2eproto__INCLUDED
