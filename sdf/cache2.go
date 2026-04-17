//-----------------------------------------------------------------------------
/*

2D Evaluation Cache

In some cases (E.g. extrusion) 2d SDFs get evaluated repeatedly at the same points.
If a map lookup is cheaper than a distance evaluation it's possible to save time
by caching evaluation results. This SDF2 wraps an underlying SDF2 and caches the
evaluations.

*/
//-----------------------------------------------------------------------------

package sdf

import (
	"fmt"
	"sync"
	"sync/atomic"

	v2 "github.com/deadsy/sdfx/vec/v2"
)

//-----------------------------------------------------------------------------

// CacheSDF2 is an SDF2 cache. Safe for concurrent use — the parallel
// marching-cubes renderer calls Evaluate from many goroutines, so the
// underlying map is protected by a RWMutex. Reads (the hit path)
// dominate, so RWMutex gives parallel readers with only the miss path
// blocking on the writer lock.
type CacheSDF2 struct {
	sdf         SDF2
	mu          sync.RWMutex
	cache       map[v2.Vec]float64
	reads, hits uint64
}

// Cache2D wraps the passed SDF2 with an evaluation cache.
func Cache2D(sdf SDF2) SDF2 {
	return &CacheSDF2{
		sdf:   sdf,
		cache: make(map[v2.Vec]float64),
	}
}

func (s *CacheSDF2) String() string {
	reads := atomic.LoadUint64(&s.reads)
	hits := atomic.LoadUint64(&s.hits)
	r := float64(hits) / float64(reads)
	return fmt.Sprintf("reads %d hits %d (%.2f)", reads, hits, r)
}

// Evaluate returns the minimum distance to a cached 2d sdf.
func (s *CacheSDF2) Evaluate(p v2.Vec) float64 {
	atomic.AddUint64(&s.reads, 1)
	s.mu.RLock()
	d, ok := s.cache[p]
	s.mu.RUnlock()
	if ok {
		atomic.AddUint64(&s.hits, 1)
		return d
	}
	d = s.sdf.Evaluate(p)
	s.mu.Lock()
	s.cache[p] = d
	s.mu.Unlock()
	return d
}

// BoundingBox returns the bounding box of a cached 2d sdf.
func (s *CacheSDF2) BoundingBox() Box2 {
	return s.sdf.BoundingBox()
}

//-----------------------------------------------------------------------------
