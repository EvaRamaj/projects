import React from 'react';
import { DataTable, TableHeader, TableBody, TableRow, TableColumn, Button } from 'react-md';

import { UserEvaluationListRow } from './UserEvaluationListRow';
import Page from './Page'


export const UserEvaluationList = ({data, onDelete}) => (
    <Page>
        <DataTable plain>
            <TableHeader>
                <TableRow>
                    <TableColumn></TableColumn>
                    <TableColumn>Name</TableColumn>
                </TableRow>
            </TableHeader>
            <TableBody>
                {data.map((user_evaluation, i) => <UserEvaluationListRow key={i} user_evaluation={user_evaluation} onDelete={(id) => onDelete(id)} />)}
            </TableBody>
        </DataTable>
    </Page>
);