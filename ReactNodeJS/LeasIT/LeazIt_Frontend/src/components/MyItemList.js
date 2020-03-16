"use strict";

import React from 'react';
import { DataTable, TableHeader, TableBody, TableRow, TableColumn, Button } from 'react-md';

import { MyItemListRow } from './MyItemListRow';
import Page from './Page'


export const MyItemList = ({data,user_evals,onDelete}) => (
    <Page>
        <div class ="row">
                {data.map((items, i) => <MyItemListRow key={i} items={items} user_evals={ user_evals} onDelete={(id) => onDelete(id)} />)}
        </div>
    </Page>
);

