using System.Linq;
using System.Collections.Generic;
using UnityEngine;


namespace Assets.Scripts.Viewer { 

    /// <summary>
    /// Manages the state of the cursor
    /// </summary>
    public static class CursorManager
    {
        /// <summary>
        /// The default cursor
        /// </summary>
        public static readonly CursorInfo Default = new CursorInfo();

        /// <summary>
        /// The list of cursors
        /// </summary>
        private static List<CursorInfo> _cursorStack = new List<CursorInfo>();

        /// <summary>
        /// The
        /// </summary>
        static CursorManager()
        {
            _cursorStack.Add(Default);
            ApplyTopCursor();
        }

        /// <summary>
        /// Gets the active cursor
        /// </summary>
        public static CursorInfo Cursor
        {
            get
            {
                return _cursorStack[0];
            }
        }

        /// <summary>
        /// Adds a cursor to the list of cursors to display
        /// </summary>
        /// <param name="cursor">The cursor to add</param>
        public static void AddCursor(CursorInfo cursor)
        {
            // Don't add the default cursor
            if (cursor != Default && cursor != null)
            {
                // By default insert at the beginning
                int insertIdx = 0;
                for (int i = 0; i < _cursorStack.Count; i++)
                {
                    CursorInfo oldCursor = _cursorStack[i];
                    if (oldCursor.Priority > cursor.Priority)
                    {
                        insertIdx = i;
                    }
                }
                _cursorStack.Insert(insertIdx, cursor);
                ApplyTopCursor();
            }
        }

        /// <summary>
        /// Removes the given cursor from display
        /// </summary>
        /// <param name="cursor">The cursor to remove</param>
        public static bool RemoveCursor(CursorInfo cursor)
        {
            bool success = false;

            // Don't remove the default cursor
            if (cursor != Default && cursor != null)
            {
                success = _cursorStack.Remove(cursor);
                if (success)
                {
                    ApplyTopCursor();
                }
            }
            return success;
        }

        /// <summary>
        /// Applies the top most cursor
        /// </summary>
        private static void ApplyTopCursor()
        {
            UnityEngine.Cursor.SetCursor(Cursor.Texture, Cursor.Hotspot, CursorMode.Auto);
        }

        /// <summary>
        /// Simple datastructure to track
        /// </summary>
        public class CursorInfo
        {
            /// <summary>
            /// Constructs a new CursorInfo
            /// </summary>
            public CursorInfo() { }

            /// <summary>
            /// Constructs a new CursorInfo
            /// </summary>
            /// <param name="texture">The cursor texture</param>
            public CursorInfo(Texture2D texture)
            {
                Texture = texture;
            }

            /// <summary>
            /// Constructs a new CursorInfo
            /// </summary>
            /// <param name="texture">The cursor texture</param>
            /// <param name="hotspot">The hotspot for the cursor</param>
            public CursorInfo(Texture2D texture, Vector2 hotspot) : this(texture)
            {
                Hotspot = hotspot;
            }

            /// <summary>
            /// Constructs a new CursorInfo
            /// </summary>
            /// <param name="texture">The cursor texture</param>
            /// <param name="hotspot">The hotspot for the cursor</param>
            public CursorInfo(Texture2D texture, Vector2 hotspot, int priority) : this(texture, hotspot)
            {
                Priority = priority;
            }

            /// <summary>
            /// The priority of the cursor
            /// </summary>
            public int Priority { get; private set; } = -1;

            /// <summary>
            /// The texture to use as the texture
            /// </summary>
            public Texture2D Texture { get; private set; } = null;

            /// <summary>
            /// The hotspot of the cursor
            /// </summary>
            public Vector2 Hotspot { get; private set; } = Vector2.zero;
        }
    }
}
