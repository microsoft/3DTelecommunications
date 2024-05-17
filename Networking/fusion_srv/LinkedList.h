//*@@@+++@@@@******************************************************************
//
// Copyright (C) Microsoft Corporation. All rights reserved.
//
//*@@@---@@@@******************************************************************

#pragma once

///////////////////////////////////////////////////////////////////////////////
// 
// Doubly linked list utility class
// 

class CLinkEntry
{
    //
    // member variables
    //
protected:
    CLinkEntry* m_next;
    CLinkEntry* m_prev;

    //
    // public interface
    //
public:
    //
    // treating this as a link head
    //
    inline bool IsEmpty ()
    {
        return (this == m_next);
    } // IsEmpty
    //
    // treating this as a link or a link head
    //
    inline CLinkEntry* Next ()
    {
        return m_next;
    } // Next
    inline CLinkEntry* Previous ()
    {
        return m_prev;
    } // Previous
    inline void InsertHead
        (
            CLinkEntry* entry
        )
    {
        entry->m_next = this->m_next;
        this->m_next->m_prev = entry;
        entry->m_prev = this;
        this->m_next = entry;
    } // InsertHead
    inline void InsertTail
        (
            CLinkEntry* entry
        )
    {
        entry->m_prev = this->m_prev;
        this->m_prev->m_next = entry;
        entry->m_next = this;
        this->m_prev = entry;
    } // InsertTail
    inline CLinkEntry* RemoveHead ()
    {
        CLinkEntry* entry = this->m_next;
        this->m_next = entry->m_next;
        entry->m_next->m_prev = this;
        entry->m_next = entry;
        entry->m_prev = entry;
        return entry;
    } // RemoveHead
    inline CLinkEntry* RemoveTail ()
    {
        CLinkEntry* entry = this->m_prev;
        this->m_prev = entry->m_prev;
        entry->m_prev->m_next = this;
        entry->m_next = entry;
        entry->m_prev = entry;
        return entry;
    } // RemoveTail
    //
    // treating this as a link
    //
    inline CLinkEntry* Remove ()
    {
        this->m_next->m_prev = this->m_prev;
        this->m_prev->m_next = this->m_next;
        m_next = this;
        m_prev = this;
        return this;
    } // Remove

    //
    // ctor
    //
public:
    CLinkEntry ()
    { 
        m_next = this;
        m_prev = this;
    } // CLinkEntry
    ~CLinkEntry ()
    { 
        Remove();
    } // CLinkEntry

    //
    // disabled
    //
private:
    CLinkEntry(const CLinkEntry& copy);
    CLinkEntry& operator=(const CLinkEntry& copy);

}; // class CLinkEntry


#define LinkEntryContainer(listentry, containertype, listentryfield) \
    ((containertype*)((BYTE*)(listentry)-(ULONG_PTR)(&((containertype*)0)->listentryfield)))
