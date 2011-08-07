#ifndef TRACKS_FILE_H
#define TRACKS_FILE_H

#include <string>
#include <vector>

class TrackEntry
{
  size_t index;
  size_t key;
  friend std::istream & operator>> (std::istream &, TrackEntry &);
  friend std::ostream & operator<< (std::ostream &, const TrackEntry &);
public:
  TrackEntry () {}
  TrackEntry (size_t index, size_t key):index(index), key(key) {}
};

class Track
{
  std::vector<TrackEntry> entries;
  friend std::istream & operator>> (std::istream &, Track &);
  friend std::ostream & operator<< (std::ostream &, const Track &);
public:
  Track () {}
  void AddEntry (const TrackEntry & entry) { entries.push_back (entry); }
};

class TracksFile
{
  std::vector<Track> tracks;
public:
  TracksFile (const std::string &);
  void AddTrack (const Track & track) { tracks.push_back (track); }
  void save (const std::string &) const;
};

#endif
