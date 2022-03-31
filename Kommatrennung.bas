Attribute VB_Name = "Modul1"
Sub KommaTrennung()

Dim AbzZeilen, i, k As Integer

Dim StartText As String

AnzZeilen = Cells(Rows.Count, 1).End(xlUp).Row

For i = 1 To AnzZeilen
    StartText = Cells(i, 1).Value
    TextArray = Split(StartText, ",")
    
    For k = LBound(TextArray, 1) To UBound(TextArray, 1)
        
        Cells(i + AnzZeilen + 1, k + 1).Value = TextArray(k)
        
    Next k

Next i

End Sub
